#ifndef TIP_OBSTACLE_H
#define TIP_OBSTACLE_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <laser_geometry/laser_geometry.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h> 

// 引入车辆状态消息、速度控制消息与可视化 Marker
#include <autoware_remove_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include <mutex>
#include <thread>
#include <atomic>
#include <string>
#include <vector>
#include <cmath>

// 1. 雷达外参结构体
struct TfParam {
    double x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;
};

// 2. 盲区过滤参数结构体
struct FilterParam {
    int left_filter_enable = 0, right_filter_enable = 0;
    double left_min_angle = 0.0, left_max_angle = 0.0;
    double left_min_y = 0.0, left_max_y = 0.0;
    double right_min_angle = 0.0, right_max_angle = 0.0;
    double right_min_y = 0.0, right_max_y = 0.0;
};

// 3. 全局应用配置结构体 (统一管理所有魔法数字)
struct AppConfig {
    double state_timeout = 2.0;
    double tf_timeout = 0.05;
    
    double reverse_velocity = -0.01;
    double forward_velocity = 0.2;
    double valid_distance_min = 0.01;
    double carport_activation_dist = 5.0;
    float max_detect_distance = 255.0f;

    std::vector<int> valid_task_types = {0, 1, 2};

    double marker_line_width = 0.05;
    double marker_color_r = 0.0;
    double marker_color_g = 1.0;
    double marker_color_b = 0.0;
    double marker_color_a = 1.0;

    FilterParam normal_filter;
    FilterParam pallet_filter;
};

class TipObstacleNode {
public:
    TipObstacleNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~TipObstacleNode();

private:
    void loadYAML();
    void watchYAMLThread();

    void palletIdCallback(const std_msgs::Int8::ConstPtr &msg);
    void scanCallbackSync(const sensor_msgs::LaserScan::ConstPtr &msg1, const sensor_msgs::LaserScan::ConstPtr &msg2);
    void scanCallbackSingle(const sensor_msgs::LaserScan::ConstPtr &msg);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filterAndTransformCloud(const sensor_msgs::LaserScan& scan_msg, bool is_left);
    Eigen::Affine3f getTransformMatrix(const TfParam& param);
    float calculateMinDisToLidar(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, bool is_left);

    // 库位动态安全走廊与可视化相关函数
    void twistCmdCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void feedbackStatusCallback(const autoware_remove_msgs::State::ConstPtr& msg);
    void applyCarportFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const ros::Time& stamp);
    void publishCarportMarker();

    ros::NodeHandle nh_, pnh_;
    
    // Publishers & Subscribers
    ros::Publisher pc_fused_pub_, pc_left_pub_, pc_right_pub_;
    ros::Publisher min_dis_pub_, carport_marker_pub_;
    ros::Subscriber pallet_id_sub_, single_scan_sub_;
    ros::Subscriber twist_cmd_sub_, feedback_status_sub_;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> sub_scan_left_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> sub_scan_right_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    laser_geometry::LaserProjection projector_;
    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener tf_listener_; 
    ros::Timer tf_timer_;

    // 基本配置参数
    int tip_type_;
    bool debug_mode_;
    std::string parent_frame_, left_child_frame_, right_child_frame_, base_link_frame_;
    
    // 三个核心 YAML 文件路径
    std::string tf_yaml_path_;
    std::string carport_yaml_path_;
    std::string config_yaml_path_;

    // 统一配置结构体与锁
    AppConfig app_cfg_;
    std::mutex cfg_mutex_;

    // 外参参数与锁
    TfParam left_tf_, right_tf_;
    std::mutex tf_mutex_;

    // 库位几何参数
    double carports_min_x_ = -0.7, carports_max_x_ = 0.9;
    double carports_min_y_ = -0.6, carports_max_y_ = 0.6;
    double carports_min_z_ = 0.0, carports_max_z_ = 1.0;

    // 文件监听线程
    std::thread yaml_watcher_thread_;
    std::atomic<bool> thread_running_;

    // 状态机变量
    std::atomic<int> pallet_id_state_{-1}; 
    std::atomic<float> dis_to_carport_{0.0f};
    std::atomic<double> last_reverse_time_{0.0}; 
    std::atomic<double> last_parking_time_{0.0}; 
    std::atomic<int> current_task_type_{-1};
    bool marker_published_{false};
};

#endif // TIP_OBSTACLE_H