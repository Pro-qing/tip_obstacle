#ifndef TIP_OBSTACLE_NEW_H
#define TIP_OBSTACLE_NEW_H

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

#include <dynamic_reconfigure/server.h>
#include <tip_obstacle_new/tip_obstacle_newConfig.h>

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

    void reconfigureCallback(tip_obstacle_new::tip_obstacle_newConfig &config, uint32_t level);
    void palletIdCallback(const std_msgs::Int8::ConstPtr &msg);
    void scanCallbackSync(const sensor_msgs::LaserScan::ConstPtr &msg1, const sensor_msgs::LaserScan::ConstPtr &msg2);
    void scanCallbackSingle(const sensor_msgs::LaserScan::ConstPtr &msg);

    // 核心函数：输入单线雷达数据，先在原始坐标系过滤，再转换到velodyne坐标系返回
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterAndTransformCloud(
        const sensor_msgs::LaserScan& scan_msg,
        bool is_left,
        float& global_min_dis);

    Eigen::Affine3f getTransformMatrix(const TfParam& param);

    ros::NodeHandle nh_, pnh_;
    
    // Publishers & Subscribers
    ros::Publisher pc_fused_pub_, min_dis_pub_;
    ros::Subscriber pallet_id_sub_, single_scan_sub_;
    
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> sub_scan_left_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> sub_scan_right_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    laser_geometry::LaserProjection projector_;
    tf::TransformBroadcaster tf_broadcaster_;
    ros::Timer tf_timer_;

    // Config variables
    int tip_type_;
    std::string parent_frame_, left_child_frame_, right_child_frame_, yaml_path_;
    std::atomic<int> pallet_id_state_; // 使用原子变量保证线程安全

    // Dynamic Reconfigure
    std::shared_ptr<dynamic_reconfigure::Server<tip_obstacle_new::tip_obstacle_newConfig>> reconfig_server_;
    tip_obstacle_new::tip_obstacle_newConfig current_cfg_;
    std::mutex cfg_mutex_;

    // TF Parameters (Loaded from YAML)
    TfParam left_tf_, right_tf_;
    std::mutex tf_mutex_;

    // Inotify Thread variables
    std::thread yaml_watcher_thread_;
    std::atomic<bool> thread_running_;
};

#endif // TIP_OBSTACLE_H