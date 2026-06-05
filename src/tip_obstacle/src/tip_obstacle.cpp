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
    pnh_.param<std::string>("left_scan_frame", left_scan_frame_, "scan_bleft_link");
    pnh_.param<std::string>("right_scan_frame", right_scan_frame_, "scan_bright_link");

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
    visibility_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visibility_region", 1, true);

    if (tip_type_ == 0) {
        sub_scan_left_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, "/scan_bleft", 1, ros::TransportHints().tcpNoDelay()));
        sub_scan_right_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, "/scan_bright", 1, ros::TransportHints().tcpNoDelay()));
        sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *sub_scan_left_, *sub_scan_right_));
        sync_->registerCallback(boost::bind(&TipObstacleNode::scanCallbackSync, this, _1, _2));
    } else {
        single_scan_sub_ = nh_.subscribe("/scan_bleft", 10, &TipObstacleNode::scanCallbackSingle, this);
    }

    // 5. 定时发布 TF 外参 (受 tf_broadcast_enable 开关控制)
    {
        AppConfig cfg;
        { std::lock_guard<std::mutex> lock(cfg_mutex_); cfg = app_cfg_; }
        if (!cfg.tf_broadcast_enable) {
            ROS_INFO("TF broadcast is DISABLED by config.");
        }
    }
    tf_timer_ = nh_.createTimer(ros::Duration(0.1), [this](const ros::TimerEvent&) {
        AppConfig cfg;
        { std::lock_guard<std::mutex> lock(cfg_mutex_); cfg = app_cfg_; }
        if (!cfg.tf_broadcast_enable) return;
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
    // 1. 读取雷达外参 TF (支持 calibration.bleft.x 和 tf_calibration.bleft_x 两种格式)
    try {
        YAML::Node config = YAML::LoadFile(tf_yaml_path_);
        std::lock_guard<std::mutex> lock(tf_mutex_);

        if (config["calibration"]) {
            // 新格式: calibration.bleft.x / calibration.bright.x
            auto cal_node = config["calibration"];
            if (cal_node["bleft"]) {
                auto bl = cal_node["bleft"];
                left_tf_.x = bl["x"].as<double>(0.0);
                left_tf_.y = bl["y"].as<double>(0.0);
                left_tf_.z = bl["z"].as<double>(0.0);
                left_tf_.roll = bl["roll"].as<double>(0.0);
                left_tf_.pitch = bl["pitch"].as<double>(0.0);
                left_tf_.yaw = bl["yaw"].as<double>(0.0);
            }
            if (cal_node["bright"]) {
                auto br = cal_node["bright"];
                right_tf_.x = br["x"].as<double>(0.0);
                right_tf_.y = br["y"].as<double>(0.0);
                right_tf_.z = br["z"].as<double>(0.0);
                right_tf_.roll = br["roll"].as<double>(0.0);
                right_tf_.pitch = br["pitch"].as<double>(0.0);
                right_tf_.yaw = br["yaw"].as<double>(0.0);
            }
        } else if (config["tf_calibration"]) {
            // 旧格式: tf_calibration.bleft_x
            auto tf_node = config["tf_calibration"];
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
        ROS_INFO("TipObstacle TF YAML Loaded Successfully.");
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
        ROS_INFO("TipObstacle Carport YAML Loaded Successfully.");
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
            app_cfg_.turning_angular_threshold = cfg["thresholds"]["turning_angular_threshold"].as<double>(0.05);
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
            app_cfg_.visibility_marker_enable = cfg["visualization"]["visibility_marker_enable"].as<int>(0);
            app_cfg_.visibility_ref_distance = cfg["visualization"]["visibility_ref_distance"].as<double>(0.5);
            app_cfg_.tf_broadcast_enable = cfg["visualization"]["tf_broadcast_enable"].as<int>(1);
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

    ROS_INFO_THROTTLE(1.0, "Received TwistCmd: vx=%.3f m/s, wz=%.3f rad/s", vx, wz);

    // 基于 YAML 中的速度阈值更新倒车时间戳
    // vx < reverse_velocity 表示后退（需要防撞），|wz| > 转向阈值表示转向（需要防撞）
    if (vx < cfg.reverse_velocity || wz > cfg.turning_angular_threshold) {
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
        ROS_INFO_THROTTLE(1.0, "Received FeedbackStatus: type=%d, distance=%.3f m", msg->TaskInfo.type, current_dis);
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

    ROS_INFO_THROTTLE(1.0, "[filterAndTransform] is_left=%d, enable_collision=%d, is_reversing=%d, "
                      "time_diff=%.3f, last_reverse_time=%.3f, raw_points=%zu",
                      is_left, enable_collision, is_reversing,
                      current_time - last_reverse_time_.load(), last_reverse_time_.load(),
                      cloud_raw->points.size());

    if (!enable_collision && !debug_mode_) {
        ROS_INFO_THROTTLE(1.0, "[filterAndTransform] Collision disabled, returning empty cloud.");
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

void TipObstacleNode::publishVisibilityMarker() {
    AppConfig cfg;
    { std::lock_guard<std::mutex> lock(cfg_mutex_); cfg = app_cfg_; }

    if (!cfg.visibility_marker_enable) return;

    visualization_msgs::MarkerArray marker_array;

    // 根据托盘状态选择对应的过滤参数
    struct VisParam {
        int filter_enable;
        double min_angle, max_angle;   // 盲区角度范围 [0, 360)
        double min_y, max_y;           // 盲区 Y 轴范围
    };

    auto getVisParam = [&](bool is_left) -> VisParam {
        VisParam vp;
        if (pallet_id_state_ > 0) {
            vp.filter_enable = is_left ? cfg.pallet_filter.left_filter_enable : cfg.pallet_filter.right_filter_enable;
            vp.min_angle     = is_left ? cfg.pallet_filter.left_min_angle : cfg.pallet_filter.right_min_angle;
            vp.max_angle     = is_left ? cfg.pallet_filter.left_max_angle : cfg.pallet_filter.right_max_angle;
            vp.min_y         = is_left ? cfg.pallet_filter.left_min_y : cfg.pallet_filter.right_min_y;
            vp.max_y         = is_left ? cfg.pallet_filter.left_max_y : cfg.pallet_filter.right_max_y;
        } else {
            vp.filter_enable = is_left ? cfg.normal_filter.left_filter_enable : cfg.normal_filter.right_filter_enable;
            vp.min_angle     = is_left ? cfg.normal_filter.left_min_angle : cfg.normal_filter.right_min_angle;
            vp.max_angle     = is_left ? cfg.normal_filter.left_max_angle : cfg.normal_filter.right_max_angle;
            vp.min_y         = is_left ? cfg.normal_filter.left_min_y : cfg.normal_filter.right_min_y;
            vp.max_y         = is_left ? cfg.normal_filter.left_max_y : cfg.normal_filter.right_max_y;
        }
        return vp;
    };

    double ref_dist = cfg.visibility_ref_distance;
    int marker_id = 0;

    // 为每个雷达生成可视区域 Marker
    // 使用 tf_listener 查找 map→scan_frame 的变换，将 Marker 发布到 map 坐标系
    auto publishForLidar = [&](bool is_left, const TfParam& tf_cfg, const std::string& scan_frame) {
        VisParam vp = getVisParam(is_left);
        if (!vp.filter_enable) return;

        // 查找 map → scan_frame 的变换
        tf::StampedTransform map_to_scan;
        try {
            tf_listener_.waitForTransform("map", scan_frame, ros::Time(0), ros::Duration(0.1));
            tf_listener_.lookupTransform("map", scan_frame, ros::Time(0), map_to_scan);
        } catch (tf::TransformException &ex) {
            ROS_WARN_THROTTLE(2.0, "Visibility Marker TF lookup failed [%s]: %s", scan_frame.c_str(), ex.what());
            return;
        }

        // 构造变换矩阵
        Eigen::Affine3f tf_map_to_scan = Eigen::Affine3f::Identity();
        tf_map_to_scan.translation() << map_to_scan.getOrigin().x(),
                                        map_to_scan.getOrigin().y(),
                                        map_to_scan.getOrigin().z();
        tf::Quaternion q = map_to_scan.getRotation();
        Eigen::Quaternionf eigen_q(q.w(), q.x(), q.y(), q.z());
        tf_map_to_scan.rotate(eigen_q);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = is_left ? "visibility_left" : "visibility_right";
        marker.id = marker_id++;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = cfg.marker_line_width;
        // 可视区域使用半透明蓝色，与库位走廊的绿色区分
        marker.color.r = 0.0;
        marker.color.g = 0.5;
        marker.color.b = 1.0;
        marker.color.a = 0.6;

        // 盲区角度 [min_angle, max_angle]，可视区域为其补集
        // 可视区域边界线: 沿 min_angle 和 max_angle 方向各画一条径向线
        // 需要考虑环绕情况: 当 min_angle > max_angle 时盲区跨 0°/360°
        // 此时可视区域为 [max_angle, min_angle] (不跨 0°)
        // 当 min_angle <= max_angle 时，可视区域为 [0, min_angle] ∪ [max_angle, 360)

        // 定义辅助函数: 角度 → map 坐标系下的 2D 点
        auto angleToPoint = [&](double angle_deg) -> geometry_msgs::Point {
            double angle_rad = angle_deg * M_PI / 180.0;
            // 雷达局部坐标系下的点
            Eigen::Vector3f local_pt(ref_dist * cos(angle_rad), ref_dist * sin(angle_rad), 0.0);
            // 变换到 map 坐标系
            Eigen::Vector3f map_pt = tf_map_to_scan * local_pt;
            geometry_msgs::Point p;
            p.x = map_pt.x();
            p.y = map_pt.y();
            p.z = map_pt.z();
            return p;
        };

        // 原点 (雷达在 map 坐标系下的位置)
        Eigen::Vector3f origin_eigen = tf_map_to_scan * Eigen::Vector3f(0.0, 0.0, 0.0);
        geometry_msgs::Point origin;
        origin.x = origin_eigen.x();
        origin.y = origin_eigen.y();
        origin.z = origin_eigen.z();

        // 计算可视角度区间并绘制径向线和弧线
        // 弧线用若干线段近似
        const int arc_segments = 8;

        auto addArcAndRays = [&](double vis_start_deg, double vis_end_deg) {
            if (vis_start_deg >= vis_end_deg) return;

            // 两条径向线 (从原点到 ref_dist)
            geometry_msgs::Point ps = angleToPoint(vis_start_deg);
            geometry_msgs::Point pe = angleToPoint(vis_end_deg);

            marker.points.push_back(origin);
            marker.points.push_back(ps);

            marker.points.push_back(origin);
            marker.points.push_back(pe);

            // 弧线 (从 start 到 end)
            for (int i = 0; i < arc_segments; ++i) {
                double a1 = vis_start_deg + (vis_end_deg - vis_start_deg) * i / arc_segments;
                double a2 = vis_start_deg + (vis_end_deg - vis_start_deg) * (i + 1) / arc_segments;
                marker.points.push_back(angleToPoint(a1));
                marker.points.push_back(angleToPoint(a2));
            }
        };

        if (vp.min_angle > vp.max_angle) {
            // 盲区跨 0°: [min_angle, 360) ∪ [0, max_angle]
            // 可视区域: [max_angle, min_angle]
            addArcAndRays(vp.max_angle, vp.min_angle);
        } else {
            // 盲区: [min_angle, max_angle]
            // 可视区域: [0, min_angle] ∪ [max_angle, 360)
            addArcAndRays(0.0, vp.min_angle);
            addArcAndRays(vp.max_angle, 360.0);
        }

        // Y 轴边界线: 沿 y=min_y 和 y=max_y 画水平线段
        // 只在 Y 轴过滤开启 (min_y != max_y) 时绘制
        if (fabs(vp.min_y - vp.max_y) > 1e-6) {
            for (double y_val : {vp.min_y, vp.max_y}) {
                Eigen::Vector3f local_p1(0.0, y_val, 0.0);
                Eigen::Vector3f local_p2(ref_dist, y_val, 0.0);
                Eigen::Vector3f map_p1 = tf_map_to_scan * local_p1;
                Eigen::Vector3f map_p2 = tf_map_to_scan * local_p2;
                geometry_msgs::Point p1, p2;
                p1.x = map_p1.x(); p1.y = map_p1.y(); p1.z = map_p1.z();
                p2.x = map_p2.x(); p2.y = map_p2.y(); p2.z = map_p2.z();
                marker.points.push_back(p1);
                marker.points.push_back(p2);
            }
        }

        if (!marker.points.empty()) {
            marker_array.markers.push_back(marker);
        }
    };

    {
        publishForLidar(true, left_tf_, left_scan_frame_);
        if (tip_type_ == 0) {
            publishForLidar(false, right_tf_, right_scan_frame_);
        }
    }

    if (!marker_array.markers.empty()) {
        visibility_marker_pub_.publish(marker_array);
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
    publishVisibilityMarker();

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
    ROS_INFO_THROTTLE(1.0, "[scanCallbackSingle] Received scan, ranges_size=%zu", msg->ranges.size());
    auto left_cloud = filterAndTransformCloud(*msg, true);
    ROS_INFO_THROTTLE(1.0, "[scanCallbackSingle] After filter: cloud_size=%zu", left_cloud->points.size());

    applyCarportFilter(left_cloud, msg->header.stamp);
    publishCarportMarker();
    publishVisibilityMarker();

    float final_min_dis = calculateMinDisToLidar(left_cloud, true);

    ROS_INFO_THROTTLE(1.0, "[scanCallbackSingle] Final: cloud_size=%zu, min_dis=%.3f", left_cloud->points.size(), final_min_dis);

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