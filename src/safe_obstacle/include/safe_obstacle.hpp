#ifndef _SAFE_OBSTACLE_HPP_
#define _SAFE_OBSTACLE_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_can_msgs/CANInfo.h>
#include <autoware_msgs/Waypoint.h>
#include <vector>

class SafeObstacle
{
public:
    SafeObstacle(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~SafeObstacle(){};

private:
    void pointsCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void twist_raw_Callback(const geometry_msgs::TwistStamped::Ptr &msg);
    void can_info_Callback(const autoware_can_msgs::CANInfo::Ptr &msg);
    void lqr_targetwayp_Callback(const autoware_msgs::Waypoint::ConstPtr &msg);

    bool PointInPolygon(const geometry_msgs::Point& point, const std::vector<geometry_msgs::Point>& polyCorner);
    pcl::PointCloud<pcl::PointXYZI> SlidingWindow_Flitter(pcl::PointCloud<pcl::PointXYZI> cloud_ptr_);
    void updateDetectionZones();
    
    visualization_msgs::Marker CreateMarker(const std::vector<geometry_msgs::Point>& polyCorner, std_msgs::ColorRGBA color_, int id_);
    std_msgs::ColorRGBA createColor(float r, float g, float b, float a) {
        std_msgs::ColorRGBA color;
        color.r = r; color.g = g; color.b = b; color.a = a;
        return color;
    }

    ros::NodeHandle nh_, pnh_;
    ros::Subscriber cloud_points_sub_, twist_raw_sub_, can_info_sub_, lqr_targetwayp_sub_;
    ros::Publisher pub_markers, pub_safe;

    geometry_msgs::TwistStamped::Ptr twist_raw_ptr_;
    autoware_can_msgs::CANInfo::Ptr can_info_ptr_;
    bool is_narrow_channel_ = false;
    bool debug_mode_ = false;

    // 基础多边形数据 (1为常规，2为窄道)
    std::vector<geometry_msgs::Point> base_exigencySize_1, base_exigencySize_2;
    std::vector<geometry_msgs::Point> base_slowSize_1, base_slowSize_2;
    std::vector<geometry_msgs::Point> base_reverse_exigencySize_1, base_reverse_exigencySize_2;
    std::vector<geometry_msgs::Point> base_reverse_slowSize_1, base_reverse_slowSize_2;

    // 当前生效的多边形数据（经过缩放后）
    std::vector<geometry_msgs::Point> current_exigencySize, current_slowSize;
    std::vector<geometry_msgs::Point> current_reverse_exigencySize, current_reverse_slowSize;

    double max_longitudinal_scale_, min_longitudinal_scale_;
    double max_lateral_scale_, min_lateral_scale_;
    double longitudinal_sensitivity_, lateral_sensitivity_, reference_speed_;
};

#endif