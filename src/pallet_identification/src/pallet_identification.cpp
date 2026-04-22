#include "pallet_identification/pallet_identification.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pallet_identification");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    PalletIdentification pi(nh, private_nh);
    ros::spin();
    return 0;
}


PalletIdentification::PalletIdentification(ros::NodeHandle& nh, ros::NodeHandle& private_nh) : nh_(nh), private_nh_(private_nh)
{
    // 从参数服务器获取阈值参数
    private_nh.param("distance_threshold", distance_threshold_, 0.5);
    private_nh.param("angle_threshold", angle_threshold_, 0.2);
    
    // 初始化订阅者和发布者
    pallet_pose_sub_ = nh_.subscribe("/pallet_pose", 10, &PalletIdentification::palletPoseCallback, this);
    ndt_pose_sub_ = nh_.subscribe("/ndt_pose", 10, &PalletIdentification::ndtPoseCallback, this);
    arrived_pub_ = nh_.advertise<std_msgs::Int8>("/arrived_flag", 10);
}

PalletIdentification::~PalletIdentification()
{
    // 析构函数
}

void PalletIdentification::palletPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pallet_pose_ = *msg;
}

void PalletIdentification::ndtPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_ndt_pose_ = *msg;
    
    std_msgs::Int8 arrived_msg;
    arrived_msg.data = 0;
    
    // 检查是否识别到托盘
    if (!isPoseValid(current_pallet_pose_)) {
        arrived_pub_.publish(arrived_msg);
        return;
    }
    
    // 计算距离差
    double dx = current_ndt_pose_.pose.position.x - current_pallet_pose_.pose.position.x;
    double dy = current_ndt_pose_.pose.position.y - current_pallet_pose_.pose.position.y;
    double dz = current_ndt_pose_.pose.position.z - current_pallet_pose_.pose.position.z;
    double distance = sqrt(dx*dx + dy*dy + dz*dz);
    
    // 计算角度差
    double ndt_yaw = getYawFromQuaternion(current_ndt_pose_.pose.orientation);
    double pallet_yaw = getYawFromQuaternion(current_pallet_pose_.pose.orientation);
    double angle_diff = fabs(ndt_yaw - pallet_yaw);
    if (angle_diff > M_PI) {
        angle_diff = 2 * M_PI - angle_diff; // 取最小角度差
    }
    
    // 检查是否满足条件
    if (distance < distance_threshold_ && angle_diff < angle_threshold_) {
        arrived_msg.data = 1;
    }

    ROS_INFO("distance:%.4f, angle_diff:%.4f (%.4f:%.4f)", distance, angle_diff, distance_threshold_, angle_threshold_);
    
    arrived_pub_.publish(arrived_msg);
}

bool PalletIdentification::isPoseValid(const geometry_msgs::PoseStamped& pose)
{
    // 检查位姿是否全为0（未识别到托盘）
    return !(pose.pose.position.x == 0 && 
             pose.pose.position.y == 0 && 
             pose.pose.position.z == 0 &&
             pose.pose.orientation.x == 0 && 
             pose.pose.orientation.y == 0 && 
             pose.pose.orientation.z == 0 && 
             pose.pose.orientation.w == 0);
}

double PalletIdentification::getYawFromQuaternion(const geometry_msgs::Quaternion& quat)
{
    tf2::Quaternion q(
        quat.x,
        quat.y,
        quat.z,
        quat.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}