#include "tip_obstacle/tip_obstacle.h"
#include <yaml-cpp/yaml.h>
#include <sys/inotify.h>
#include <poll.h>
#include <unistd.h>
#include <algorithm>

TipObstacleNode::TipObstacleNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) 
    : nh_(nh), pnh_(pnh), thread_running_(true), base_link_frame_("base_link")
{
    // 1. 获取基础 ROS 参数
    pnh_.param<bool>("debug", debug_mode_, false);
    if (debug_mode_) {
        ROS_WARN("\n=======================================================\n"
                 " TIP_OBSTACLE DEBUG MODE IS ENABLED! \n"
                 " All logic filters (distance, angle, carport) are BYPASSED.\n"
                 " Publishing RAW separated & fused point clouds to Velodyne!\n"
                 "=======================================================");
    }

    pnh_.param<int>("tip_type", tip_type_, 0);
    pnh_.param<std::string>("parent_frame", parent_frame_, "velodyne");
    pnh_.param<std::string>("left_child_frame", left_child_frame_, "bleft_laser");
    pnh_.param<std::string>("right_child_frame", right_child_frame_, "bright_laser");

    std::string out_fused_pc, out_left_pc, out_right_pc;
    pnh_.param<std::string>("out_fused_points_cloud", out_fused_pc, "fused_points_tip");
    pnh_.param<std::string>("out_bleft_points_cloud", out_left_pc, "bleft_points_tip");
    pnh_.param<std::string>("out_bright_points_cloud", out_right_pc, "bright_points_tip");

    // 2. 获取三个核心 YAML 路径
    pnh_.param<std::string>("tf_yaml_path", tf_yaml_path_, "");
    pnh_.param<std::string>("carport_yaml_path", carport_yaml_path_, "");
    pnh_.param<std::string>("config_yaml_path", config_yaml_path_, "");

    // 3. 加载配置文件并启动监听线程
    loadYAML();
    yaml_watcher_thread_ = std::thread(&TipObstacleNode::watchYAMLThread, this);

    // 4. 设置 ROS 通信
    pc_fused_pub_  = nh_.advertise<sensor_msgs::PointCloud2>(out_fused_pc, 10);
    pc_left_pub_   = nh_.advertise<sensor_msgs::PointCloud2>(out_left_pc, 10);
    pc_right_pub_  = nh_.advertise<sensor_msgs::PointCloud2>(out_right_pc, 10);
    min_dis_pub_   = nh_.advertise<std_msgs::Float32>("tip_dis", 10);
    
    pallet_id_sub_ = nh_.subscribe("/arrived_flag", 10, &TipObstacleNode::palletIdCallback, this);
    twist_cmd_sub_ = nh_.subscribe("/twist_cmd", 10, &TipObstacleNode::twistCmdCallback, this);
    feedback_status_sub_ = nh_.subscribe("/feedback_status", 10, &TipObstacleNode::feedbackStatusCallback, this);
    carport_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("carport", 1, true);

    if (tip_type_ == 0) {
        sub_scan_left_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, "/scan_bleft", 1, ros::TransportHints().tcpNoDelay()));
        sub_scan_right_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, "/scan_bright", 1, ros::TransportHints().tcpNoDelay()));
        sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *sub_scan_left_, *sub_scan_right_));
        sync_->registerCallback(boost::bind(&TipObstacleNode::scanCallbackSync, this, _1, _2));
    } else {
        single_scan_sub_ = nh_.subscribe("/scan_bleft", 10, &TipObstacleNode::scanCallbackSingle, this);
    }

    // 5. 定时发布 TF 外参
    tf_timer_ = nh_.createTimer(ros::Duration(0.1), [this](const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lock(tf_mutex_);
        ros::Time now = ros::Time::now();

        tf::Transform tf_left;
        tf_left.setOrigin(tf::Vector3(left_tf_.x, left_tf_.y, left_tf_.z));
        tf::Quaternion q_left;
        q_left.setRPY(left_tf_.roll, left_tf_.pitch, left_tf_.yaw);
        tf_left.setRotation(q_left);
        tf_broadcaster_.sendTransform(tf::StampedTransform(tf_left, now, parent_frame_, left_child_frame_));

        tf::Transform tf_right;
        tf_right.setOrigin(tf::Vector3(right_tf_.x, right_tf_.y, right_tf_.z));
        tf::Quaternion q_right;
        q_right.setRPY(right_tf_.roll, right_tf_.pitch, right_tf_.yaw);
        tf_right.setRotation(q_right);
        tf_broadcaster_.sendTransform(tf::StampedTransform(tf_right, now, parent_frame_, right_child_frame_));
    });
}

TipObstacleNode::~TipObstacleNode() {
    thread_running_ = false;
    if (yaml_watcher_thread_.joinable()) yaml_watcher_thread_.join();
}

void TipObstacleNode::loadYAML() {
    // 1. 读取雷达外参 TF
    try {
        YAML::Node config = YAML::LoadFile(tf_yaml_path_);
        if (config["tf_calibration"]) {
            auto tf_node = config["tf_calibration"];
            std::lock_guard<std::mutex> lock(tf_mutex_);
            left_tf_.x = tf_node["bleft_x"].as<double>(0.0);
            left_tf_.y = tf_node["bleft_y"].as<double>(0.0);
            left_tf_.z = tf_node["bleft_z"].as<double>(0.0);
            left_tf_.yaw = tf_node["bleft_yaw"].as<double>(0.0);
            left_tf_.pitch = tf_node["bleft_pitch"].as<double>(0.0);
            left_tf_.roll = tf_node["bleft_roll"].as<double>(0.0);

            right_tf_.x = tf_node["bright_x"].as<double>(0.0);
            right_tf_.y = tf_node["bright_y"].as<double>(0.0);
            right_tf_.z = tf_node["bright_z"].as<double>(0.0);
            right_tf_.yaw = tf_node["bright_yaw"].as<double>(0.0);
            right_tf_.pitch = tf_node["bright_pitch"].as<double>(0.0);
            right_tf_.roll = tf_node["bright_roll"].as<double>(0.0);
        }
    } catch (const YAML::Exception& e) {
        ROS_ERROR("Failed to load Lidar TF YAML: %s", e.what());
    }

    // 2. 读取库位尺寸参数
    try {
        YAML::Node carport_config = YAML::LoadFile(carport_yaml_path_);
        if (carport_config["carports_min_x"]) {
            std::lock_guard<std::mutex> lock(cfg_mutex_);
            carports_min_x_ = carport_config["carports_min_x"].as<double>();
            carports_max_x_ = carport_config["carports_max_x"].as<double>();
            carports_min_y_ = carport_config["carports_min_y"].as<double>();
            carports_max_y_ = carport_config["carports_max_y"].as<double>();
            carports_min_z_ = carport_config["carports_min_z"].as<double>();
            carports_max_z_ = carport_config["carports_max_z"].as<double>();
        }
    } catch (const YAML::Exception& e) {
        ROS_WARN("Failed to load Carport YAML: %s", e.what());
    }

    // 3. 读取全局防撞策略与过滤配置 (替换魔法数字)
    try {
        YAML::Node cfg = YAML::LoadFile(config_yaml_path_);
        std::lock_guard<std::mutex> lock(cfg_mutex_);

        if (cfg["timeouts"]) {
            app_cfg_.state_timeout = cfg["timeouts"]["state_timeout"].as<double>(2.0);
            app_cfg_.tf_timeout = cfg["timeouts"]["tf_timeout"].as<double>(0.05);
        }
        if (cfg["thresholds"]) {
            app_cfg_.reverse_velocity = cfg["thresholds"]["reverse_velocity"].as<double>(-0.01);
            app_cfg_.forward_velocity = cfg["thresholds"]["forward_velocity"].as<double>(0.2);
            app_cfg_.valid_distance_min = cfg["thresholds"]["valid_distance_min"].as<double>(0.01);
            app_cfg_.carport_activation_dist = cfg["thresholds"]["carport_activation_dist"].as<double>(5.0);
            app_cfg_.max_detect_distance = cfg["thresholds"]["max_detect_distance"].as<float>(255.0f);
        }
        if (cfg["valid_task_types"]) {
            app_cfg_.valid_task_types = cfg["valid_task_types"].as<std::vector<int>>();
        }
        if (cfg["visualization"]) {
            app_cfg_.marker_line_width = cfg["visualization"]["marker_line_width"].as<double>(0.05);
            app_cfg_.marker_color_r = cfg["visualization"]["marker_color_r"].as<double>(0.0);
            app_cfg_.marker_color_g = cfg["visualization"]["marker_color_g"].as<double>(1.0);
            app_cfg_.marker_color_b = cfg["visualization"]["marker_color_b"].as<double>(0.0);
            app_cfg_.marker_color_a = cfg["visualization"]["marker_color_a"].as<double>(1.0);
        }
        if (cfg["normal_filter"]) {
            auto nf = cfg["normal_filter"];
            app_cfg_.normal_filter.left_filter_enable = nf["left_filter_enable"].as<int>(0);
            app_cfg_.normal_filter.left_min_angle = nf["left_min_angle"].as<double>(0.0);
            app_cfg_.normal_filter.left_max_angle = nf["left_max_angle"].as<double>(0.0);
            app_cfg_.normal_filter.left_min_y = nf["left_min_y"].as<double>(0.0);
            app_cfg_.normal_filter.left_max_y = nf["left_max_y"].as<double>(0.0);
            app_cfg_.normal_filter.right_filter_enable = nf["right_filter_enable"].as<int>(0);
            app_cfg_.normal_filter.right_min_angle = nf["right_min_angle"].as<double>(0.0);
            app_cfg_.normal_filter.right_max_angle = nf["right_max_angle"].as<double>(0.0);
            app_cfg_.normal_filter.right_min_y = nf["right_min_y"].as<double>(0.0);
            app_cfg_.normal_filter.right_max_y = nf["right_max_y"].as<double>(0.0);
        }
        if (cfg["pallet_id_filter"]) {
            auto pf = cfg["pallet_id_filter"];
            app_cfg_.pallet_filter.left_filter_enable = pf["pallet_id_left_filter_enable"].as<int>(0);
            app_cfg_.pallet_filter.left_min_angle = pf["pallet_id_left_min_angle"].as<double>(0.0);
            app_cfg_.pallet_filter.left_max_angle = pf["pallet_id_left_max_angle"].as<double>(0.0);
            app_cfg_.pallet_filter.left_min_y = pf["pallet_id_left_min_y"].as<double>(0.0);
            app_cfg_.pallet_filter.left_max_y = pf["pallet_id_left_max_y"].as<double>(0.0);
            app_cfg_.pallet_filter.right_filter_enable = pf["pallet_id_right_filter_enable"].as<int>(0);
            app_cfg_.pallet_filter.right_min_angle = pf["pallet_id_right_min_angle"].as<double>(0.0);
            app_cfg_.pallet_filter.right_max_angle = pf["pallet_id_right_max_angle"].as<double>(0.0);
            app_cfg_.pallet_filter.right_min_y = pf["pallet_id_right_min_y"].as<double>(0.0);
            app_cfg_.pallet_filter.right_max_y = pf["pallet_id_right_max_y"].as<double>(0.0);
        }
        ROS_INFO("TipObstacle Config YAML Loaded Successfully.");
    } catch (const YAML::Exception& e) {
        ROS_WARN("Failed to load TipObstacle Config YAML: %s", e.what());
    }
}

void TipObstacleNode::watchYAMLThread() {
    int fd = inotify_init1(IN_NONBLOCK);
    if (fd < 0) return;

    // 添加三个文件的监听
    auto add_watch = [&](const std::string& path, std::string& file_name) -> int {
        if (path.empty()) return -1;
        size_t last_slash = path.find_last_of('/');
        std::string dir_path = path.substr(0, last_slash);
        file_name = path.substr(last_slash + 1);
        return inotify_add_watch(fd, dir_path.c_str(), IN_CLOSE_WRITE | IN_MOVED_TO);
    };

    std::string file_name1, file_name2, file_name3;
    int wd1 = add_watch(tf_yaml_path_, file_name1);
    int wd2 = add_watch(carport_yaml_path_, file_name2);
    int wd3 = add_watch(config_yaml_path_, file_name3);

    pollfd pfd = {fd, POLLIN, 0};

    while (thread_running_) {
        int ret = poll(&pfd, 1, 500); 
        if (ret > 0 && (pfd.revents & POLLIN)) {
            char buffer[4096] __attribute__ ((aligned(__alignof__(struct inotify_event))));
            ssize_t len = read(fd, buffer, sizeof(buffer));
            for (char *ptr = buffer; ptr < buffer + len; ) {
                struct inotify_event *event = (struct inotify_event *) ptr;
                if (event->len) {
                    std::string ev_name(event->name);
                    if (ev_name == file_name1 || ev_name == file_name2 || ev_name == file_name3) {
                        ROS_WARN("YAML File [%s] changed, reloading all parameters...", event->name);
                        loadYAML();
                    }
                }
                ptr += sizeof(struct inotify_event) + event->len;
            }
        }
    }
    if (wd1 >= 0) inotify_rm_watch(fd, wd1);
    if (wd2 >= 0) inotify_rm_watch(fd, wd2);
    if (wd3 >= 0) inotify_rm_watch(fd, wd3);
    close(fd);
}

void TipObstacleNode::palletIdCallback(const std_msgs::Int8::ConstPtr &msg) {
    pallet_id_state_ = msg->data;
}

void TipObstacleNode::twistCmdCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    AppConfig cfg;
    { std::lock_guard<std::mutex> lock(cfg_mutex_); cfg = app_cfg_; }

    float vx = msg->twist.linear.x;
    float wz = fabs(msg->twist.angular.z);

    // 基于 YAML 中的速度阈值更新倒车时间戳
    if (vx < cfg.reverse_velocity || wz > 0) {
        last_reverse_time_.store(ros::Time::now().toSec());
    } 
    else if (vx > cfg.forward_velocity) {
        last_reverse_time_.store(0.0);
    }
}

void TipObstacleNode::feedbackStatusCallback(const autoware_remove_msgs::State::ConstPtr& msg) {
    AppConfig cfg;
    { std::lock_guard<std::mutex> lock(cfg_mutex_); cfg = app_cfg_; }

    current_task_type_.store(msg->TaskInfo.type);

    // 检查任务类型是否在 YAML 允许的数组中
    auto& v_types = cfg.valid_task_types;
    if (std::find(v_types.begin(), v_types.end(), msg->TaskInfo.type) != v_types.end()) {
        float current_dis = msg->TaskInfo.site.dis;
        
        if (current_dis > cfg.valid_distance_min) {
            dis_to_carport_.store(current_dis); 
            // 判断是否到达长廊激活距离
            if (current_dis < cfg.carport_activation_dist) {
                last_parking_time_.store(ros::Time::now().toSec());
            }
        }
    } 
}

Eigen::Affine3f TipObstacleNode::getTransformMatrix(const TfParam& param) {
    Eigen::Affine3f mat = Eigen::Affine3f::Identity();
    mat.translation() << param.x, param.y, param.z;
    mat.rotate(Eigen::AngleAxisf(param.yaw, Eigen::Vector3f::UnitZ()));
    mat.rotate(Eigen::AngleAxisf(param.pitch, Eigen::Vector3f::UnitY()));
    mat.rotate(Eigen::AngleAxisf(param.roll, Eigen::Vector3f::UnitX()));
    return mat;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr TipObstacleNode::filterAndTransformCloud(
    const sensor_msgs::LaserScan& scan_msg, bool is_left) 
{
    sensor_msgs::PointCloud2 pc2_msg;
    projector_.projectLaser(scan_msg, pc2_msg);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(pc2_msg, *cloud_raw);

    AppConfig cfg;
    { std::lock_guard<std::mutex> lock(cfg_mutex_); cfg = app_cfg_; }

    // 【防撞核心开关】：通过 YAML 配置的超时时间进行判断
    double current_time = ros::Time::now().toSec();
    bool is_reversing = (current_time - last_reverse_time_.load()) < cfg.state_timeout;
    bool enable_collision = is_reversing; 

    if (!enable_collision && !debug_mode_) {
        return cloud_filtered; // 返回空点云
    }

    // 根据托盘状态选择对应的 YAML 过滤配置
    bool enable_filter = false;
    double min_ang = 0, max_ang = 0, min_y = 0, max_y = 0;

    if (pallet_id_state_ > 0) {
        enable_filter = is_left ? (cfg.pallet_filter.left_filter_enable == 1) : (cfg.pallet_filter.right_filter_enable == 1);
        min_ang = is_left ? cfg.pallet_filter.left_min_angle : cfg.pallet_filter.right_min_angle;
        max_ang = is_left ? cfg.pallet_filter.left_max_angle : cfg.pallet_filter.right_max_angle;
        min_y   = is_left ? cfg.pallet_filter.left_min_y : cfg.pallet_filter.right_min_y;
        max_y   = is_left ? cfg.pallet_filter.left_max_y : cfg.pallet_filter.right_max_y;
    } else {
        enable_filter = is_left ? (cfg.normal_filter.left_filter_enable == 1) : (cfg.normal_filter.right_filter_enable == 1);
        min_ang = is_left ? cfg.normal_filter.left_min_angle : cfg.normal_filter.right_min_angle;
        max_ang = is_left ? cfg.normal_filter.left_max_angle : cfg.normal_filter.right_max_angle;
        min_y   = is_left ? cfg.normal_filter.left_min_y : cfg.normal_filter.right_min_y;
        max_y   = is_left ? cfg.normal_filter.left_max_y : cfg.normal_filter.right_max_y;
    }

    // 盲区过滤计算
    for (const auto& pt : cloud_raw->points) {
        if (!debug_mode_ && enable_filter) {
            double angle = atan2(pt.y, pt.x);
            angle = fmod((angle * 180.0 / M_PI) + 360.0, 360.0);
            
            bool is_in_blind_angle = false;
            if (min_ang > max_ang) {
                if (angle > min_ang || angle < max_ang) is_in_blind_angle = true;
            } else {
                if (angle > min_ang && angle < max_ang) is_in_blind_angle = true;
            }
            if (is_in_blind_angle) continue;

            bool is_in_blind_y = false;
            if (min_y > max_y) {
                if (pt.y > min_y || pt.y < max_y) is_in_blind_y = true;
            } else {
                if (pt.y > min_y && pt.y < max_y) is_in_blind_y = true;
            }
            if (is_in_blind_y) continue;
        } 
        cloud_filtered->points.push_back(pt);
    }

    // TF 外参变换
    TfParam tf_cfg;
    {
        std::lock_guard<std::mutex> lock(tf_mutex_);
        tf_cfg = is_left ? left_tf_ : right_tf_;
    }
    pcl::transformPointCloud(*cloud_filtered, *cloud_filtered, getTransformMatrix(tf_cfg));

    return cloud_filtered;
}

float TipObstacleNode::calculateMinDisToLidar(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, bool is_left) {
    AppConfig cfg;
    { std::lock_guard<std::mutex> lock(cfg_mutex_); cfg = app_cfg_; }
    
    // 使用 YAML 中的最远距离作为初始值
    float min_dis = cfg.max_detect_distance; 
    
    TfParam tf_cfg;
    {
        std::lock_guard<std::mutex> lock(tf_mutex_);
        tf_cfg = is_left ? left_tf_ : right_tf_;
    }

    for (const auto& pt : cloud->points) {
        float dx = pt.x - tf_cfg.x;
        float dy = pt.y - tf_cfg.y;
        float dz = pt.z - tf_cfg.z;
        float r = sqrt(dx * dx + dy * dy + dz * dz);
        if (r < min_dis) {
            min_dis = r;
        }
    }
    return min_dis;
}

void TipObstacleNode::applyCarportFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const ros::Time& stamp) {
    if (debug_mode_) return;
    
    AppConfig cfg;
    { std::lock_guard<std::mutex> lock(cfg_mutex_); cfg = app_cfg_; }

    double current_time = ros::Time::now().toSec();
    bool is_reversing = (current_time - last_reverse_time_.load()) < cfg.state_timeout;
    bool is_parking   = (current_time - last_parking_time_.load()) < cfg.state_timeout;

    if (!is_reversing || !is_parking) return;

    bool need_tf_transform = (parent_frame_ != base_link_frame_);
    Eigen::Affine3f tf_velodyne_to_baselink = Eigen::Affine3f::Identity();
    
    if (need_tf_transform) {
        tf::StampedTransform transform;
        try {
            // 使用 YAML 中配置的 TF 等待超时时间
            tf_listener_.waitForTransform(base_link_frame_, parent_frame_, stamp, ros::Duration(cfg.tf_timeout));
            tf_listener_.lookupTransform(base_link_frame_, parent_frame_, stamp, transform);
        } catch (tf::TransformException &ex) {
            ROS_WARN_THROTTLE(1.0, "Carport TF time sync failed: %s", ex.what());
            return;
        }

        tf_velodyne_to_baselink.translation() << transform.getOrigin().x(), 
                                                 transform.getOrigin().y(), 
                                                 transform.getOrigin().z();
        tf::Quaternion q = transform.getRotation();
        Eigen::Quaternionf eigen_q(q.w(), q.x(), q.y(), q.z());
        tf_velodyne_to_baselink.rotate(eigen_q);

        pcl::transformPointCloud(*cloud, *cloud, tf_velodyne_to_baselink);
    }

    float current_dis = dis_to_carport_.load();
    pcl::PointCloud<pcl::PointXYZI>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZI>());

    double min_x, max_x, min_y, max_y, min_z, max_z;
    {
        std::lock_guard<std::mutex> lock(cfg_mutex_);
        min_x = carports_min_x_; max_x = carports_max_x_;
        min_y = carports_min_y_; max_y = carports_max_y_;
        min_z = carports_min_z_; max_z = carports_max_z_;
    }

    double cx = (min_x + max_x) / 2.0;
    double cy = (min_y + max_y) / 2.0;

    double abs_min_x = -current_dis + (min_x - cx);
    double original_abs_max_x = -current_dis + (max_x - cx);
    double abs_max_x = std::max(original_abs_max_x, 0.0); 

    double abs_min_y = min_y - cy;
    double abs_max_y = max_y - cy;
    double abs_min_z = min_z;
    double abs_max_z = max_z;

    for (const auto& pt : cloud->points) {
        if (pt.x >= abs_min_x && pt.x <= abs_max_x &&
            pt.y >= abs_min_y && pt.y <= abs_max_y &&
            pt.z >= abs_min_z && pt.z <= abs_max_z) 
        {
            cropped_cloud->points.push_back(pt);
        }
    }

    cropped_cloud->width = cropped_cloud->points.size();
    cropped_cloud->height = 1;
    cropped_cloud->is_dense = true;

    if (need_tf_transform) {
        Eigen::Affine3f tf_baselink_to_velodyne = tf_velodyne_to_baselink.inverse();
        pcl::transformPointCloud(*cropped_cloud, *cloud, tf_baselink_to_velodyne);
    } else {
        *cloud = *cropped_cloud;
    }
}

void TipObstacleNode::publishCarportMarker() {
    AppConfig cfg;
    { std::lock_guard<std::mutex> lock(cfg_mutex_); cfg = app_cfg_; }

    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker box_marker;

    box_marker.header.frame_id = base_link_frame_;
    box_marker.header.stamp = ros::Time::now();
    box_marker.ns = "carport_boundary";
    box_marker.id = 0;

    double current_time = ros::Time::now().toSec();
    bool is_reversing = (current_time - last_reverse_time_.load()) < cfg.state_timeout;
    bool is_parking   = (current_time - last_parking_time_.load()) < cfg.state_timeout;

    if (!is_parking || !is_reversing) {
        if (marker_published_) {
            box_marker.action = visualization_msgs::Marker::DELETE;
            marker_array.markers.push_back(box_marker);
            carport_marker_pub_.publish(marker_array);
            marker_published_ = false; 
        }
        return;
    }

    marker_published_ = true;

    box_marker.type = visualization_msgs::Marker::LINE_LIST;
    box_marker.action = visualization_msgs::Marker::ADD;
    box_marker.pose.orientation.w = 1.0;
    
    // 使用 YAML 中配置的可视化参数
    box_marker.scale.x = cfg.marker_line_width; 
    box_marker.color.r = cfg.marker_color_r;
    box_marker.color.g = cfg.marker_color_g;
    box_marker.color.b = cfg.marker_color_b;
    box_marker.color.a = cfg.marker_color_a;

    float current_dis = dis_to_carport_.load();
    double min_x, max_x, min_y, max_y, min_z, max_z;
    {
        std::lock_guard<std::mutex> lock(cfg_mutex_);
        min_x = carports_min_x_; max_x = carports_max_x_;
        min_y = carports_min_y_; max_y = carports_max_y_;
        min_z = carports_min_z_; max_z = carports_max_z_;
    }

    double cx = (min_x + max_x) / 2.0;
    double cy = (min_y + max_y) / 2.0;

    double abs_min_x = -current_dis + (min_x - cx);
    double original_abs_max_x = -current_dis + (max_x - cx);
    double abs_max_x = std::max(original_abs_max_x, 0.0); 
    
    double abs_min_y = min_y - cy;
    double abs_max_y = max_y - cy;
    double abs_min_z = min_z; 
    double abs_max_z = max_z; 

    geometry_msgs::Point p[8];
    p[0].x = abs_min_x; p[0].y = abs_min_y; p[0].z = abs_min_z;
    p[1].x = abs_max_x; p[1].y = abs_min_y; p[1].z = abs_min_z;
    p[2].x = abs_max_x; p[2].y = abs_max_y; p[2].z = abs_min_z;
    p[3].x = abs_min_x; p[3].y = abs_max_y; p[3].z = abs_min_z;
    
    p[4].x = abs_min_x; p[4].y = abs_min_y; p[4].z = abs_max_z;
    p[5].x = abs_max_x; p[5].y = abs_min_y; p[5].z = abs_max_z;
    p[6].x = abs_max_x; p[6].y = abs_max_y; p[6].z = abs_max_z;
    p[7].x = abs_min_x; p[7].y = abs_max_y; p[7].z = abs_max_z;

    int edges[12][2] = {
        {0,1}, {1,2}, {2,3}, {3,0},
        {4,5}, {5,6}, {6,7}, {7,4},
        {0,4}, {1,5}, {2,6}, {3,7} 
    };

    for (int i = 0; i < 12; ++i) {
        box_marker.points.push_back(p[edges[i][0]]);
        box_marker.points.push_back(p[edges[i][1]]);
    }

    marker_array.markers.push_back(box_marker);
    carport_marker_pub_.publish(marker_array);
}

void TipObstacleNode::scanCallbackSync(const sensor_msgs::LaserScan::ConstPtr &msg1,
                                       const sensor_msgs::LaserScan::ConstPtr &msg2) 
{
    auto left_cloud = filterAndTransformCloud(*msg1, true);
    auto right_cloud = filterAndTransformCloud(*msg2, false);

    applyCarportFilter(left_cloud, msg1->header.stamp);
    applyCarportFilter(right_cloud, msg2->header.stamp);

    publishCarportMarker();

    float min_dis_left = calculateMinDisToLidar(left_cloud, true);
    float min_dis_right = calculateMinDisToLidar(right_cloud, false);
    float final_min_dis = std::min(min_dis_left, min_dis_right);

    sensor_msgs::PointCloud2 leftOutMsg, rightOutMsg;
    pcl::toROSMsg(*left_cloud, leftOutMsg);
    leftOutMsg.header.frame_id = parent_frame_;
    leftOutMsg.header.stamp = msg1->header.stamp;
    pc_left_pub_.publish(leftOutMsg);

    pcl::toROSMsg(*right_cloud, rightOutMsg);
    rightOutMsg.header.frame_id = parent_frame_;
    rightOutMsg.header.stamp = msg2->header.stamp;
    pc_right_pub_.publish(rightOutMsg);

    *left_cloud += *right_cloud;
    sensor_msgs::PointCloud2 fusedOutMsg;
    pcl::toROSMsg(*left_cloud, fusedOutMsg);
    fusedOutMsg.header.frame_id = parent_frame_;
    fusedOutMsg.header.stamp = msg1->header.stamp;
    pc_fused_pub_.publish(fusedOutMsg);

    std_msgs::Float32 dis_msg;
    dis_msg.data = final_min_dis;
    min_dis_pub_.publish(dis_msg);
}

void TipObstacleNode::scanCallbackSingle(const sensor_msgs::LaserScan::ConstPtr &msg) 
{
    auto left_cloud = filterAndTransformCloud(*msg, true);

    applyCarportFilter(left_cloud, msg->header.stamp);
    publishCarportMarker();

    float final_min_dis = calculateMinDisToLidar(left_cloud, true);

    sensor_msgs::PointCloud2 leftOutMsg;
    pcl::toROSMsg(*left_cloud, leftOutMsg);
    leftOutMsg.header.frame_id = parent_frame_;
    leftOutMsg.header.stamp = msg->header.stamp;
    pc_left_pub_.publish(leftOutMsg);

    sensor_msgs::PointCloud2 fusedOutMsg = leftOutMsg;
    pc_fused_pub_.publish(fusedOutMsg);

    std_msgs::Float32 dis_msg;
    dis_msg.data = final_min_dis;
    min_dis_pub_.publish(dis_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tip_obstacle");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    TipObstacleNode node(nh, pnh);

    ros::spin();
    return 0;
}