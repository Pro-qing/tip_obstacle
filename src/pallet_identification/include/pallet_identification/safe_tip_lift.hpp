#ifndef __SAFE_TIP_LIFT_HPP
#define __SAFE_TIP_LIFT_HPP

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt16MultiArray.h>
#include <autoware_can_msgs/CANInfo.h>
#include <autoware_msgs/VehicleCmd.h>
#include <autoware_remove_msgs/State.h>
#include <cmath>
#include <algorithm>

class SafeTipLift{
public:
    SafeTipLift();
    
    ~SafeTipLift();

private:
    ros::Subscriber can_info_sub_, vehicle_cmd_sub_;
    ros::Subscriber arrived_flag_sub_;
    ros::Subscriber feedback_status_sub_;
    ros::Publisher safetiplift_pub_;

    uint16_t base_def_safelift_;
    uint16_t base_pallet_safelift_;
    
    // 角度调整参数
    double max_angle_;          // 最大考虑角度（弧度）
    double min_adjustment_;     // 角度最大时的最小调整系数
    double max_adjustment_;     // 角度为0时的最大调整系数
    double smoothness_factor_;  // 平滑因子，控制曲线形状
    
    int8_t arrived_flag_;      // 到达标志
    double current_angle_;     // 当前转向角（弧度）
    uint16_t actual_lift_;     // 实际叉尖距离（从/feedback_status获取）
    bool inplace_;
    double current_speed_;
    
    // State消息指针
    autoware_remove_msgs::State::ConstPtr state_ptr_;

    // CAN信息回调函数
    void canInfoCallback(const autoware_can_msgs::CANInfo::ConstPtr& msg);
    void vehicleCmdCallback(const autoware_msgs::VehicleCmd::ConstPtr& msg);
    
    // 到达标志回调函数
    void arrivedFlagCallback(const std_msgs::Int8::ConstPtr& msg);
    
    // 反馈状态回调函数
    void feedbackStatusCallback(const autoware_remove_msgs::State::ConstPtr& msg);
    
    // 发布安全叉尖距离
    void publishSafeTipLift();
    
    // 使用反向平滑函数计算安全距离（角度越大，安全距离越小）
    uint16_t calculateInverseSafeLift(uint16_t base_safelift, double angle_rad);
    
    // 备选方案：使用反向S形曲线实现更平滑的过渡
    uint16_t calculateInverseSigmoidSafeLift(uint16_t base_safelift, double angle_rad);
    
    // 备选方案：使用倒数函数实现角度越大安全距离越小
    uint16_t calculateReciprocalSafeLift(uint16_t base_safelift, double angle_rad);
};

#endif