#include "tip_obstacle/tip_obstacle.h"
#include <yaml-cpp/yaml.h>
#include <sys/inotify.h>
#include <poll.h>
#include <unistd.h>
#include <algorithm>

TipObstacleNode::TipObstacleNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) 
    : nh_(nh), pnh_(pnh), thread_running_(true), pallet_id_state_(-1), 
      last_reverse_time_(0), last_parking_time_(0)
{
    // 1. 获取常规参数与 Debug 模式
    pnh_.param<bool>("debug", debug_mode_, false);

    // 读取库位长廊激活距离，如果没有配置则默认使用 8.0 米 
    pnh_.param<double>("carport_activation_dist", carport_activation_dist_, 8.0);
    if (debug_mode_) {
        ROS_WARN("\n=======================================================\n"
                 " TIP_OBSTACLE DEBUG MODE IS ENABLED! \n"
                 " All filters (distance, angle, carport) are BYPASSED.\n"
                 " Publishing RAW separated & fused point clouds to Velodyne!\n"
                 "=======================================================");
    }

    pnh_.param<int>("tip_type", tip_type_, 0);
    pnh_.param<std::string>("parent_frame", parent_frame_, "velodyne");
    pnh_.param<std::string>("left_child_frame", left_child_frame_, "bleft_laser");
    pnh_.param<std::string>("right_child_frame", right_child_frame_, "bright_laser");
    pnh_.param<std::string>("base_link_frame", base_link_frame_, "base_link");

    // 获取发布话题名称
    std::string out_fused_pc, out_left_pc, out_right_pc;
    pnh_.param<std::string>("out_fused_points_cloud", out_fused_pc, "fused_points_tip");
    pnh_.param<std::string>("out_left_points_cloud", out_left_pc, "left_points_tip");
    pnh_.param<std::string>("out_right_points_cloud", out_right_pc, "right_points_tip");

    // 2. 获取 YAML 路径参数 (TF 和 库位)
    // const char* home_dir = getenv("HOME");
    // std::string default_yaml_path_tf = (home_dir ? std::string(home_dir) : "") + "/work/workspace/param/lidar_calibration.yaml";
    // pnh_.param<std::string>("tf_yaml_path", yaml_path_, default_yaml_path_tf);
    // ROS_INFO("TipObstacle: Using Lidar YAML from: %s", yaml_path_.c_str());

    // std::string default_yaml_path_carport = (home_dir ? std::string(home_dir) : "") + "/work/autoware.ai/src/ant/secure_vehicle/obstacle_detection/params/obstacle_detection.yaml";
    // pnh_.param<std::string>("carport_yaml_path", carport_yaml_path_, default_yaml_path_carport);
    // ROS_INFO("TipObstacle: Using Carport YAML from: %s", carport_yaml_path_.c_str());

    std::string default_yaml_path_tf = "/home/ros_ws/workspace/param/lidar_calibration.yaml";
    pnh_.param<std::string>("tf_yaml_path", yaml_path_, default_yaml_path_tf);
    ROS_INFO("TipObstacle: Using Lidar YAML from: %s", yaml_path_.c_str());

    std::string default_yaml_path_carport = "/home/ros_ws/workspace/config/perception/obstacle_detection.yaml";
    pnh_.param<std::string>("carport_yaml_path", carport_yaml_path_, default_yaml_path_carport);
    ROS_INFO("TipObstacle: Using Carport YAML from: %s", carport_yaml_path_.c_str());
    

    // 3. 加载配置文件并启动双文件监听线程
    loadYAML();
    yaml_watcher_thread_ = std::thread(&TipObstacleNode::watchYAMLThread, this);

    // 4. 初始化动态参数服务器
    reconfig_server_ = std::make_shared<dynamic_reconfigure::Server<tip_obstacle::tip_obstacleConfig>>(pnh_);
    reconfig_server_->setCallback(boost::bind(&TipObstacleNode::reconfigureCallback, this, _1, _2));

    // 5. 设置 ROS 通信
    pc_fused_pub_  = nh_.advertise<sensor_msgs::PointCloud2>(out_fused_pc, 10);
    pc_left_pub_   = nh_.advertise<sensor_msgs::PointCloud2>(out_left_pc, 10);
    pc_right_pub_  = nh_.advertise<sensor_msgs::PointCloud2>(out_right_pc, 10);
    min_dis_pub_   = nh_.advertise<std_msgs::Float32>("tip_dis", 10);
    
    pallet_id_sub_ = nh_.subscribe("/arrived_flag", 10, &TipObstacleNode::palletIdCallback, this);
    twist_cmd_sub_ = nh_.subscribe("/twist_cmd", 10, &TipObstacleNode::twistCmdCallback, this);
    feedback_status_sub_ = nh_.subscribe("/feedback_status", 10, &TipObstacleNode::feedbackStatusCallback, this);
    carport_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("carport", 1, true);

    // 根据类型初始化雷达订阅
    if (tip_type_ == 0) {
        sub_scan_left_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, "/scan_bleft", 1, ros::TransportHints().tcpNoDelay()));
        sub_scan_right_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, "/scan_bright", 1, ros::TransportHints().tcpNoDelay()));
        sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *sub_scan_left_, *sub_scan_right_));
        sync_->registerCallback(boost::bind(&TipObstacleNode::scanCallbackSync, this, _1, _2));
    } else {
        single_scan_sub_ = nh_.subscribe("/scan_bleft", 10, &TipObstacleNode::scanCallbackSingle, this);
    }

    // 6. 启动 TF 定时广播 (10Hz)
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
    // 读取雷达外参 TF
    try {
        YAML::Node config = YAML::LoadFile(yaml_path_);
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
        ROS_ERROR("Failed to load Lidar YAML: %s", e.what());
    }

    // 读取库位尺寸参数
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
        ROS_WARN("Failed to load Carport YAML (will use defaults): %s", e.what());
    }
}

void TipObstacleNode::watchYAMLThread() {
    int fd = inotify_init1(IN_NONBLOCK);
    if (fd < 0) return;

    size_t last_slash1 = yaml_path_.find_last_of('/');
    std::string dir_path1 = yaml_path_.substr(0, last_slash1);
    std::string file_name1 = yaml_path_.substr(last_slash1 + 1);
    int wd1 = inotify_add_watch(fd, dir_path1.c_str(), IN_CLOSE_WRITE | IN_MOVED_TO);

    size_t last_slash2 = carport_yaml_path_.find_last_of('/');
    std::string dir_path2 = carport_yaml_path_.substr(0, last_slash2);
    std::string file_name2 = carport_yaml_path_.substr(last_slash2 + 1);
    int wd2 = inotify_add_watch(fd, dir_path2.c_str(), IN_CLOSE_WRITE | IN_MOVED_TO);

    pollfd pfd = {fd, POLLIN, 0};

    while (thread_running_) {
        int ret = poll(&pfd, 1, 500); 
        if (ret > 0 && (pfd.revents & POLLIN)) {
            char buffer[4096] __attribute__ ((aligned(__alignof__(struct inotify_event))));
            ssize_t len = read(fd, buffer, sizeof(buffer));
            for (char *ptr = buffer; ptr < buffer + len; ) {
                struct inotify_event *event = (struct inotify_event *) ptr;
                if (event->len) {
                    if (file_name1 == event->name || file_name2 == event->name) {
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
    close(fd);
}

void TipObstacleNode::reconfigureCallback(tip_obstacle::tip_obstacleConfig &config, uint32_t level) {
    std::lock_guard<std::mutex> lock(cfg_mutex_);
    current_cfg_ = config;
}

void TipObstacleNode::palletIdCallback(const std_msgs::Int8::ConstPtr &msg) {
    pallet_id_state_ = msg->data;
}

// 速度回调：只要速度小于 -0.01，就更新时间戳
void TipObstacleNode::twistCmdCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    float vx = msg->twist.linear.x;
    
    // 只要有微弱的倒车速度，立刻续命 2 秒
    if (vx < -0.01) {
        last_reverse_time_.store(ros::Time::now().toSec());
    } 
    // 只有当车辆明确挂入前进挡，且大脚油门开走时 (>0.2m/s)
    // 直接清零时间戳，瞬间解除倒车过滤状态
    else if (vx > 0.2) {
        last_reverse_time_.store(0.0);
    }
}

// 任务回调：保护合法距离，防止 0.0 污染导致绿框坍缩回车底
void TipObstacleNode::feedbackStatusCallback(const autoware_remove_msgs::State::ConstPtr& msg) {
    // 1. 实时更新当前的任务类型
    current_task_type_.store(msg->TaskInfo.type);

    // 2. 只有卸货任务 (type == 1)，才去解析距离
    if (msg->TaskInfo.type == 1) {
        float current_dis = msg->TaskInfo.site.dis;
        
        // 过滤掉上游异常的 0.0 距离
        if (current_dis > 0.01f) {
            dis_to_carport_.store(current_dis); 
            
            // 【核心】：只有距离小于 8.0 米，才被认定为“抵达库位长廊”，开始刷新长廊时间锁
            if (current_dis < carport_activation_dist_) {
                last_parking_time_.store(ros::Time::now().toSec());
            }
        }
    } 
    // 其他任务，或者距离 > 8米时，时间锁都不会更新，2秒后长廊过滤自然关闭。
}

Eigen::Affine3f TipObstacleNode::getTransformMatrix(const TfParam& param) {
    Eigen::Affine3f mat = Eigen::Affine3f::Identity();
    mat.translation() << param.x, param.y, param.z;
    mat.rotate(Eigen::AngleAxisf(param.yaw, Eigen::Vector3f::UnitZ()));
    mat.rotate(Eigen::AngleAxisf(param.pitch, Eigen::Vector3f::UnitY()));
    mat.rotate(Eigen::AngleAxisf(param.roll, Eigen::Vector3f::UnitX()));
    return mat;
}

// 删除了 float& global_min_dis 参数
pcl::PointCloud<pcl::PointXYZI>::Ptr TipObstacleNode::filterAndTransformCloud(
    const sensor_msgs::LaserScan& scan_msg, bool is_left) 
{
    sensor_msgs::PointCloud2 pc2_msg;
    projector_.projectLaser(scan_msg, pc2_msg);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(pc2_msg, *cloud_raw);

    // ================== 【核心防撞开关逻辑】 ==================
    int task_type = current_task_type_.load();
    double current_time = ros::Time::now().toSec();
    bool is_reversing = (current_time - last_reverse_time_.load()) < 2.0;
    
    bool enable_collision = false; 
    if (task_type == 1) {
        enable_collision = is_reversing;
    } else {
        enable_collision = is_reversing;
    }

    if (!enable_collision && !debug_mode_) {
        return cloud_filtered; 
    }
    // ==========================================================

    tip_obstacle::tip_obstacleConfig cfg;
    {
        std::lock_guard<std::mutex> lock(cfg_mutex_);
        cfg = current_cfg_;
    }

    bool enable_filter = false;
    double min_ang = 0, max_ang = 0, min_y = 0, max_y = 0;

    if (pallet_id_state_ > 0) {
        enable_filter = is_left ? cfg.pallet_id_left_filter_enable : cfg.pallet_id_right_filter_enable;
        min_ang = is_left ? cfg.pallet_id_left_min_angle : cfg.pallet_id_right_min_angle;
        max_ang = is_left ? cfg.pallet_id_left_max_angle : cfg.pallet_id_right_max_angle;
        min_y = is_left ? cfg.pallet_id_left_min_y : cfg.pallet_id_right_min_y;
        max_y = is_left ? cfg.pallet_id_left_max_y : cfg.pallet_id_right_max_y;
    } else {
        enable_filter = is_left ? cfg.left_filter_enable : cfg.right_filter_enable;
        min_ang = is_left ? cfg.left_min_angle : cfg.right_min_angle;
        max_ang = is_left ? cfg.left_max_angle : cfg.right_max_angle;
        min_y = is_left ? cfg.left_min_y : cfg.right_min_y;
        max_y = is_left ? cfg.left_max_y : cfg.right_max_y;
    }

    // ================== 【步骤1：过滤点云】 ==================
    for (const auto& pt : cloud_raw->points) {
        float r = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);

        if (!debug_mode_) {
            if (cfg.r_dis_enable == 1) {
                if (r < cfg.r_dis_min || r > cfg.r_dis_max) continue; 
            }
            if (enable_filter) {
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
        } 
        cloud_filtered->points.push_back(pt);
    }

    // ================== 【步骤3：执行 TF 变换】 ==================
    TfParam tf_cfg;
    {
        std::lock_guard<std::mutex> lock(tf_mutex_);
        tf_cfg = is_left ? left_tf_ : right_tf_;
    }
    pcl::transformPointCloud(*cloud_filtered, *cloud_filtered, getTransformMatrix(tf_cfg));

    return cloud_filtered;
}

// 新增函数：在最终的融合/剪裁完毕后的点云中，计算点到雷达几何原点的最小真实距离
float TipObstacleNode::calculateMinDisToLidar(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, bool is_left) {
    float min_dis = 255.0f;
    TfParam tf_cfg;
    {
        // 线程安全地获取当前计算的是左雷达还是右雷达的外参原点
        std::lock_guard<std::mutex> lock(tf_mutex_);
        tf_cfg = is_left ? left_tf_ : right_tf_;
    }

    // cloud 当前处于 parent_frame_ (例如 velodyne) 坐标系中
    // 所以点到雷达的真实距离，就是该点坐标减去雷达自身平移位置 (tf_cfg.x, tf_cfg.y, tf_cfg.z) 之后的欧氏距离
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


// 核心：基于精确时间戳与防抖状态机的动态安全走廊裁剪
void TipObstacleNode::applyCarportFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const ros::Time& stamp) {
    if (debug_mode_) return;
    // 【终极状态机判断】：当前时间减去最后续命时间，是否小于 2.0 秒？
    double current_time = ros::Time::now().toSec();
    bool is_reversing = (current_time - last_reverse_time_.load()) < 2.0;
    bool is_parking   = (current_time - last_parking_time_.load()) < 2.0;

    // 只要有任何一个超时了，就说明不在倒车入库状态，不予裁剪
    if (!is_reversing || !is_parking) return;

    bool need_tf_transform = (parent_frame_ != base_link_frame_);
    Eigen::Affine3f tf_velodyne_to_baselink = Eigen::Affine3f::Identity();
    
    if (need_tf_transform) {
        tf::StampedTransform transform;
        try {
            // 【时间戳极致对齐】：强制等待点云发出的那一刻的 TF，消灭幽灵错位点！
            tf_listener_.waitForTransform(base_link_frame_, parent_frame_, stamp, ros::Duration(0.05));
            tf_listener_.lookupTransform(base_link_frame_, parent_frame_, stamp, transform);
        } catch (tf::TransformException &ex) {
            ROS_WARN_THROTTLE(1.0, "Carport TF time sync failed, fallback to latest: %s", ex.what());
            try {
                tf_listener_.lookupTransform(base_link_frame_, parent_frame_, ros::Time(0), transform);
            } catch (tf::TransformException &ex2) {
                return; 
            }
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

// 在 Rviz 中发布与裁剪区域完全一致的动态安全走廊（绿色 3D 线框）
void TipObstacleNode::publishCarportMarker() {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker box_marker;

    box_marker.header.frame_id = base_link_frame_; // 始终固定在车辆中心坐标系
    box_marker.header.stamp = ros::Time::now();
    box_marker.ns = "carport_boundary";
    box_marker.id = 0;

    // 【核心抗抖动机制】：通过最后一次接收到有效状态的时间戳，判断是否在 2 秒内
    double current_time = ros::Time::now().toSec();
    bool is_reversing = (current_time - last_reverse_time_.load()) < 2.0;
    bool is_parking   = (current_time - last_parking_time_.load()) < 2.0;

    // 如果不在入库状态，或者已经超过 2 秒没有倒车动作，则清除 RViz 里的绿框
    if (!is_parking || !is_reversing) {
        // 只有当屏幕上确实有框时，才发送一次 DELETE，避免 10Hz 狂刷导致渲染闪烁
        if (marker_published_) {
            box_marker.action = visualization_msgs::Marker::DELETE;
            marker_array.markers.push_back(box_marker);
            carport_marker_pub_.publish(marker_array);
            marker_published_ = false; // 标记屏幕上已经没有框了
        }
        return;
    }

    // 运行到这里，说明处于稳定的倒车入库状态，准备画框
    marker_published_ = true;

    box_marker.type = visualization_msgs::Marker::LINE_LIST;
    box_marker.action = visualization_msgs::Marker::ADD;
    box_marker.pose.orientation.w = 1.0;
    box_marker.scale.x = 0.05; // 线条的粗细

    // 设定颜色为绿色 (R:0, G:1, B:0, A:1)
    box_marker.color.r = 0.0;
    box_marker.color.g = 1.0;
    box_marker.color.b = 0.0;
    box_marker.color.a = 1.0;

    // 获取当前的入库距离
    float current_dis = dis_to_carport_.load();
    
    // 获取 YAML 中的库位尺寸参数
    double min_x, max_x, min_y, max_y, min_z, max_z;
    {
        std::lock_guard<std::mutex> lock(cfg_mutex_);
        min_x = carports_min_x_; 
        max_x = carports_max_x_;
        min_y = carports_min_y_; 
        max_y = carports_max_y_;
        min_z = carports_min_z_; 
        max_z = carports_max_z_;
    }

    // 计算 YAML 库位尺寸在 XY 平面上的几何中心，用于将库位强行对齐到 base_link 轴线
    double cx = (min_x + max_x) / 2.0;
    double cy = (min_y + max_y) / 2.0;
    // Z 轴通常不需要对齐中心，直接使用配置的高度即可

    // 【动态安全走廊计算核心】
    // 库位最深处的 X 坐标（倒车方向，即车尾后方 current_dis 加上库位深度一半）
    double abs_min_x = -current_dis + (min_x - cx);
    
    // 原本框的最前端 X 坐标
    double original_abs_max_x = -current_dis + (max_x - cx);
    
    // 将框的最前端强行拉伸到车尾中心 (X = 0.0)，形成一条保护车身到库位之间所有空间的“走廊”
    double abs_max_x = std::max(original_abs_max_x, 0.0); 
    
    // Y 和 Z 轴的绝对边界（Z轴直接使用配置值，使框贴地或悬空特定高度）
    double abs_min_y = min_y - cy;
    double abs_max_y = max_y - cy;
    double abs_min_z = min_z; 
    double abs_max_z = max_z; 

    // 根据 6 个极限坐标，定义长方体的 8 个顶点
    geometry_msgs::Point p[8];
    // 底面 4 个点 (z = min_z)
    p[0].x = abs_min_x; p[0].y = abs_min_y; p[0].z = abs_min_z;
    p[1].x = abs_max_x; p[1].y = abs_min_y; p[1].z = abs_min_z;
    p[2].x = abs_max_x; p[2].y = abs_max_y; p[2].z = abs_min_z;
    p[3].x = abs_min_x; p[3].y = abs_max_y; p[3].z = abs_min_z;
    
    // 顶面 4 个点 (z = max_z)
    p[4].x = abs_min_x; p[4].y = abs_min_y; p[4].z = abs_max_z;
    p[5].x = abs_max_x; p[5].y = abs_min_y; p[5].z = abs_max_z;
    p[6].x = abs_max_x; p[6].y = abs_max_y; p[6].z = abs_max_z;
    p[7].x = abs_min_x; p[7].y = abs_max_y; p[7].z = abs_max_z;

    // 定义长方体的 12 条边（每条边由两个点的索引组成，用于 LINE_LIST 绘制）
    int edges[12][2] = {
        {0,1}, {1,2}, {2,3}, {3,0}, // 底面边框
        {4,5}, {5,6}, {6,7}, {7,4}, // 顶面边框
        {0,4}, {1,5}, {2,6}, {3,7}  // 四根垂直柱子
    };

    // 将 12 条边的 24 个点压入 marker
    for (int i = 0; i < 12; ++i) {
        box_marker.points.push_back(p[edges[i][0]]);
        box_marker.points.push_back(p[edges[i][1]]);
    }

    // 发布这个绿色 3D 线框
    marker_array.markers.push_back(box_marker);
    carport_marker_pub_.publish(marker_array);
}

void TipObstacleNode::scanCallbackSync(const sensor_msgs::LaserScan::ConstPtr &msg1,
                                       const sensor_msgs::LaserScan::ConstPtr &msg2) 
{
    // 调用修改后的函数，仅做盲区过滤和 TF 变换，暂不计算距离
    auto left_cloud = filterAndTransformCloud(*msg1, true);
    auto right_cloud = filterAndTransformCloud(*msg2, false);

    // 【极高精度时间戳对齐裁剪】在计算距离之前执行完毕！
    applyCarportFilter(left_cloud, msg1->header.stamp);
    applyCarportFilter(right_cloud, msg2->header.stamp);

    publishCarportMarker();

    // 【核心修复】：在经过所有的裁剪器（盲区+距离+长廊）之后，基于剩下的有效点，计算点到对应雷达的距离！
    float min_dis_left = calculateMinDisToLidar(left_cloud, true);
    float min_dis_right = calculateMinDisToLidar(right_cloud, false);

    // 取左右雷达的最小值作为最终输出
    float final_min_dis = std::min(min_dis_left, min_dis_right);

    // 分离发布：左雷达 & 右雷达点云
    sensor_msgs::PointCloud2 leftOutMsg, rightOutMsg;
    pcl::toROSMsg(*left_cloud, leftOutMsg);
    leftOutMsg.header.frame_id = parent_frame_;
    leftOutMsg.header.stamp = msg1->header.stamp;
    pc_left_pub_.publish(leftOutMsg);

    pcl::toROSMsg(*right_cloud, rightOutMsg);
    rightOutMsg.header.frame_id = parent_frame_;
    rightOutMsg.header.stamp = msg2->header.stamp;
    pc_right_pub_.publish(rightOutMsg);

    // 融合发布
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
    // 同理，去掉了传递引用的距离参数
    auto left_cloud = filterAndTransformCloud(*msg, true);

    // 裁剪点云
    applyCarportFilter(left_cloud, msg->header.stamp);
    publishCarportMarker();

    // 在全部过滤完毕后，算出真实距离
    float final_min_dis = calculateMinDisToLidar(left_cloud, true);

    // 分离发布：左雷达点云
    sensor_msgs::PointCloud2 leftOutMsg;
    pcl::toROSMsg(*left_cloud, leftOutMsg);
    leftOutMsg.header.frame_id = parent_frame_;
    leftOutMsg.header.stamp = msg->header.stamp;
    pc_left_pub_.publish(leftOutMsg);

    // 融合发布（与左点云相同）
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