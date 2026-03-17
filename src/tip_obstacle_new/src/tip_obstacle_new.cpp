#include "tip_obstacle_new/tip_obstacle_new.h"
#include <yaml-cpp/yaml.h>
#include <sys/inotify.h>
#include <poll.h>
#include <unistd.h>

TipObstacleNode::TipObstacleNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) 
    : nh_(nh), pnh_(pnh), thread_running_(true), pallet_id_state_(-1)
{
    pnh_.param<int>("tip_type", tip_type_, 0);
    pnh_.param<std::string>("parent_frame", parent_frame_, "velodyne");
    pnh_.param<std::string>("left_child_frame", left_child_frame_, "bleft_laser");
    pnh_.param<std::string>("right_child_frame", right_child_frame_, "bright_laser");

    std::string out_fused_pc;
    pnh_.param<std::string>("out_fused_points_cloud", out_fused_pc, "fused_points_tip");

    // 解析 YAML 路径
    const char* home_dir = getenv("HOME");
    std::string base_path = home_dir ? std::string(home_dir) : "";
    yaml_path_ = base_path + "/work/workspace/param/lidar_calibration.yaml";

    loadYAML();
    yaml_watcher_thread_ = std::thread(&TipObstacleNode::watchYAMLThread, this);

    // 动态参数服务器
    reconfig_server_ = std::make_shared<dynamic_reconfigure::Server<tip_obstacle_new::tip_obstacle_newConfig>>(pnh_);
    reconfig_server_->setCallback(boost::bind(&TipObstacleNode::reconfigureCallback, this, _1, _2));

    // ROS 通信
    pc_fused_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(out_fused_pc, 10);
    min_dis_pub_ = nh_.advertise<std_msgs::Float32>("tip_dis", 10);
    pallet_id_sub_ = nh_.subscribe("/arrived_flag", 10, &TipObstacleNode::palletIdCallback, this);

    if (tip_type_ == 0) {
        sub_scan_left_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, "/scan_bleft", 1, ros::TransportHints().tcpNoDelay()));
        sub_scan_right_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, "/scan_bright", 1, ros::TransportHints().tcpNoDelay()));
        sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *sub_scan_left_, *sub_scan_right_));
        sync_->registerCallback(boost::bind(&TipObstacleNode::scanCallbackSync, this, _1, _2));
    } else {
        single_scan_sub_ = nh_.subscribe("/scan_bleft", 10, &TipObstacleNode::scanCallbackSingle, this);
    }

    // 定时发送单独的左右雷达 TF 到主雷达
    tf_timer_ = nh_.createTimer(ros::Duration(0.1), [this](const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lock(tf_mutex_);
        ros::Time now = ros::Time::now();

        // 1. 发送 左雷达 -> velodyne
        tf::Transform tf_left;
        tf_left.setOrigin(tf::Vector3(left_tf_.x, left_tf_.y, left_tf_.z));
        tf::Quaternion q_left;
        q_left.setRPY(left_tf_.roll, left_tf_.pitch, left_tf_.yaw);
        tf_left.setRotation(q_left);
        tf_broadcaster_.sendTransform(tf::StampedTransform(tf_left, now, parent_frame_, left_child_frame_));

        // 2. 发送 右雷达 -> velodyne
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
    if (yaml_watcher_thread_.joinable()) {
        yaml_watcher_thread_.join();
    }
}

void TipObstacleNode::loadYAML() {
    try {
        YAML::Node config = YAML::LoadFile(yaml_path_);
        if (config["tf_calibration"]) {
            auto tf_node = config["tf_calibration"];
            std::lock_guard<std::mutex> lock(tf_mutex_);
            
            // 左后 -> Left
            left_tf_.x = tf_node["bleft_x"].as<double>(0.0);
            left_tf_.y = tf_node["bleft_y"].as<double>(0.0);
            left_tf_.z = tf_node["bleft_z"].as<double>(0.0);
            left_tf_.yaw = tf_node["bleft_yaw"].as<double>(0.0);
            left_tf_.pitch = tf_node["bleft_pitch"].as<double>(0.0);
            left_tf_.roll = tf_node["bleft_roll"].as<double>(0.0);

            // 右后 -> Right
            right_tf_.x = tf_node["bright_x"].as<double>(0.0);
            right_tf_.y = tf_node["bright_y"].as<double>(0.0);
            right_tf_.z = tf_node["bright_z"].as<double>(0.0);
            right_tf_.yaw = tf_node["bright_yaw"].as<double>(0.0);
            right_tf_.pitch = tf_node["bright_pitch"].as<double>(0.0);
            right_tf_.roll = tf_node["bright_roll"].as<double>(0.0);

            ROS_INFO("Loaded TF Calibration from YAML Successfully!");
        }
    } catch (const YAML::Exception& e) {
        ROS_ERROR("Failed to load YAML: %s", e.what());
    }
}

void TipObstacleNode::watchYAMLThread() {
    int fd = inotify_init1(IN_NONBLOCK);
    if (fd < 0) return;

    size_t last_slash = yaml_path_.find_last_of('/');
    std::string dir_path = yaml_path_.substr(0, last_slash);
    std::string file_name = yaml_path_.substr(last_slash + 1);

    int wd = inotify_add_watch(fd, dir_path.c_str(), IN_CLOSE_WRITE | IN_MOVED_TO);
    pollfd pfd = {fd, POLLIN, 0};

    while (thread_running_) {
        int ret = poll(&pfd, 1, 500); 
        if (ret > 0 && (pfd.revents & POLLIN)) {
            char buffer[4096] __attribute__ ((aligned(__alignof__(struct inotify_event))));
            ssize_t len = read(fd, buffer, sizeof(buffer));
            for (char *ptr = buffer; ptr < buffer + len; ) {
                struct inotify_event *event = (struct inotify_event *) ptr;
                if (event->len && file_name == event->name) {
                    ROS_WARN("YAML File changed, reloading...");
                    loadYAML();
                }
                ptr += sizeof(struct inotify_event) + event->len;
            }
        }
    }
    inotify_rm_watch(fd, wd);
    close(fd);
}

void TipObstacleNode::reconfigureCallback(tip_obstacle_new::tip_obstacle_newConfig &config, uint32_t level) {
    std::lock_guard<std::mutex> lock(cfg_mutex_);
    current_cfg_ = config;
}

void TipObstacleNode::palletIdCallback(const std_msgs::Int8::ConstPtr &msg) {
    pallet_id_state_ = msg->data;
}

Eigen::Affine3f TipObstacleNode::getTransformMatrix(const TfParam& param) {
    Eigen::Affine3f mat = Eigen::Affine3f::Identity();
    mat.translation() << param.x, param.y, param.z;
    mat.rotate(Eigen::AngleAxisf(param.yaw, Eigen::Vector3f::UnitZ()));
    mat.rotate(Eigen::AngleAxisf(param.pitch, Eigen::Vector3f::UnitY()));
    mat.rotate(Eigen::AngleAxisf(param.roll, Eigen::Vector3f::UnitX()));
    return mat;
}

// 核心处理：1.单线转点云 2.在原始坐标系过滤 3.转换到velodyne
pcl::PointCloud<pcl::PointXYZI>::Ptr TipObstacleNode::filterAndTransformCloud(
    const sensor_msgs::LaserScan& scan_msg, bool is_left, float& global_min_dis) 
{
    // 1. 单线雷达数据转换为点云数据 (此时点云位于雷达局部原始坐标系)
    sensor_msgs::PointCloud2 pc2_msg;
    projector_.projectLaser(scan_msg, pc2_msg);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(pc2_msg, *cloud_raw);

    tip_obstacle_new::tip_obstacle_newConfig cfg;
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

    // 2. 原始坐标系下的数据过滤（角度、Y轴、最短距离记录）
    for (const auto& pt : cloud_raw->points) {
        if (enable_filter) {
            double angle = atan2(pt.y, pt.x);
            angle = fmod((angle * 180.0 / M_PI) + 360.0, 360.0);

            if (min_ang > max_ang) {
                if (angle > min_ang || angle < max_ang) continue;
            } else {
                if (angle > min_ang && angle < max_ang) continue;
            }

            if (min_y > max_y) {
                if (pt.y > min_y || pt.y < max_y) continue;
            } else {
                if (pt.y > min_y && pt.y < max_y) continue;
            }
        }

        // 仅在原始数据上计算绝对距离
        float r = sqrt(pt.x * pt.x + pt.y * pt.y);
        if (r < fabs(global_min_dis)) {
            global_min_dis = r;
        }
        cloud_filtered->points.push_back(pt);
    }

    // 3. 将过滤后的局部点云变换到主雷达 (velodyne) 坐标系
    TfParam tf_cfg;
    {
        std::lock_guard<std::mutex> lock(tf_mutex_);
        tf_cfg = is_left ? left_tf_ : right_tf_;
    }
    pcl::transformPointCloud(*cloud_filtered, *cloud_filtered, getTransformMatrix(tf_cfg));

    return cloud_filtered; // 返回的已经是 velodyne 坐标系下的数据
}

void TipObstacleNode::scanCallbackSync(const sensor_msgs::LaserScan::ConstPtr &msg1,
                                       const sensor_msgs::LaserScan::ConstPtr &msg2) 
{
    float min_dis = 255.0f;
    
    // 获取的均是基于velodyne坐标系的过滤点云
    auto left_cloud_in_velodyne = filterAndTransformCloud(*msg1, true, min_dis);
    auto right_cloud_in_velodyne = filterAndTransformCloud(*msg2, false, min_dis);

    // 点云拼接融合
    *left_cloud_in_velodyne += *right_cloud_in_velodyne;

    // 发布融合后的点云
    sensor_msgs::PointCloud2 fusedOutMsg;
    pcl::toROSMsg(*left_cloud_in_velodyne, fusedOutMsg);
    fusedOutMsg.header.frame_id = parent_frame_; // velodyne
    fusedOutMsg.header.stamp = msg1->header.stamp;
    pc_fused_pub_.publish(fusedOutMsg);

    // 发布最小距离
    std_msgs::Float32 dis_msg;
    dis_msg.data = min_dis;
    min_dis_pub_.publish(dis_msg);
}

void TipObstacleNode::scanCallbackSingle(const sensor_msgs::LaserScan::ConstPtr &msg) 
{
    float min_dis = 255.0f;
    auto left_cloud_in_velodyne = filterAndTransformCloud(*msg, true, min_dis);

    sensor_msgs::PointCloud2 fusedOutMsg;
    pcl::toROSMsg(*left_cloud_in_velodyne, fusedOutMsg);
    fusedOutMsg.header.frame_id = parent_frame_; // velodyne
    fusedOutMsg.header.stamp = msg->header.stamp;
    pc_fused_pub_.publish(fusedOutMsg);

    std_msgs::Float32 dis_msg;
    dis_msg.data = min_dis;
    min_dis_pub_.publish(dis_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tip_obstacle_new");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    TipObstacleNode node(nh, pnh);

    ros::spin();
    return 0;
}