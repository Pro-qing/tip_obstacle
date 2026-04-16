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

#include <dynamic_reconfigure/server.h>
#include <tip_obstacle/tip_obstacleConfig.h>

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

struct TfParam {
    double x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;
};

class TipObstacleNode {
public:
    TipObstacleNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~TipObstacleNode();

private:
    void loadYAML();
    void watchYAMLThread();

    void reconfigureCallback(tip_obstacle::tip_obstacleConfig &config, uint32_t level);
    void palletIdCallback(const std_msgs::Int8::ConstPtr &msg);
    void scanCallbackSync(const sensor_msgs::LaserScan::ConstPtr &msg1, const sensor_msgs::LaserScan::ConstPtr &msg2);
    void scanCallbackSingle(const sensor_msgs::LaserScan::ConstPtr &msg);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filterAndTransformCloud(
            const sensor_msgs::LaserScan& scan_msg,
            bool is_left);

    Eigen::Affine3f getTransformMatrix(const TfParam& param);
    float calculateMinDisToLidar(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, bool is_left);

    // --- 库位动态安全走廊与可视化相关函数 ---
    void twistCmdCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void feedbackStatusCallback(const autoware_remove_msgs::State::ConstPtr& msg);
    // 增加时间戳参数，实现极高精度的 TF 空间对齐
    void applyCarportFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const ros::Time& stamp);
    void publishCarportMarker();

    ros::NodeHandle nh_, pnh_;
    
    // Publishers & Subscribers
    ros::Publisher pc_fused_pub_, pc_left_pub_, pc_right_pub_; // 独立及融合点云发布
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

    // Config variables
    int tip_type_;
    bool debug_mode_; // 调试模式标志位
    std::string parent_frame_, left_child_frame_, right_child_frame_, yaml_path_;
    std::string base_link_frame_; // 车辆几何中心坐标系 (base_link)
    std::atomic<int> pallet_id_state_; 

    // Dynamic Reconfigure
    std::shared_ptr<dynamic_reconfigure::Server<tip_obstacle::tip_obstacleConfig>> reconfig_server_;
    tip_obstacle::tip_obstacleConfig current_cfg_;
    std::mutex cfg_mutex_;

    // TF Parameters (Loaded from YAML)
    TfParam left_tf_, right_tf_;
    std::mutex tf_mutex_;

    // Inotify Thread variables
    std::thread yaml_watcher_thread_;
    std::atomic<bool> thread_running_;

    // --- 库位参数与状态机变量 ---
    std::string carport_yaml_path_;
    std::atomic<float> dis_to_carport_{0.0f};
    
    // 改为记录时间戳（单位：秒）
    std::atomic<double> last_reverse_time_{0.0}; 
    std::atomic<double> last_parking_time_{0.0}; 
    bool marker_published_{false};

    // 实时记录上游下发的任务类型
    std::atomic<int> current_task_type_{-1};

    // 从 launch 文件读取的库位长廊激活距离参数 
    double carport_activation_dist_;

    // 库位默认尺寸参数
    double carports_min_x_ = -0.7;
    double carports_max_x_ = 0.9;
    double carports_min_y_ = -0.6;
    double carports_max_y_ = 0.6;
    double carports_min_z_ = 0.0;
    double carports_max_z_ = 1.0;
};

#endif // TIP_OBSTACLE_H
