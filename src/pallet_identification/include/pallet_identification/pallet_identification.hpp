#ifndef __PALLET_IDENTIFICATION_HPP
#define __PALLET_IDENTIFICATION_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class PalletIdentification
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    ros::Subscriber pallet_pose_sub_;
    ros::Subscriber ndt_pose_sub_;
    ros::Publisher arrived_pub_;
    
    geometry_msgs::PoseStamped current_pallet_pose_;
    geometry_msgs::PoseStamped current_ndt_pose_;
    
    double distance_threshold_;
    double angle_threshold_;
    
    void palletPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void ndtPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    bool isPoseValid(const geometry_msgs::PoseStamped& pose);
    double getYawFromQuaternion(const geometry_msgs::Quaternion& quat);
    
public:
    PalletIdentification(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    ~PalletIdentification();
};

#endif