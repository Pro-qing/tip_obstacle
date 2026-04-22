#include <yaml-cpp/yaml.h>
#include "pallet_identification/safe_tip_lift.hpp"
#include <autoware_remove_msgs/State.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "safe_tip_lift");
    SafeTipLift safetiplift;
    ros::spin();
    return 0;
}

SafeTipLift::SafeTipLift() {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    std::string config_path;
    if (!private_nh.getParam("config_file", config_path)) {
        ROS_FATAL("Missing 'config_file' parameter");
        ros::shutdown();
        return;
    }

    YAML::Node config = YAML::LoadFile(config_path);
        
    base_def_safelift_ = config["base_def_safelift"].as<uint16_t>(30);
    base_pallet_safelift_ = config["base_pallet_safelift"].as<uint16_t>(20);
    max_angle_ = config["max_angle"].as<double>(1.5708);
    min_adjustment_ = config["min_adjustment"].as<double>(0.5);
    max_adjustment_ = config["max_adjustment"].as<double>(1);
    smoothness_factor_ = config["smoothness_factor"].as<double>(2.0);

    
    
    // 初始化订阅者和发布者
    can_info_sub_ = nh.subscribe<autoware_can_msgs::CANInfo>("/can_info", 10, 
                    &SafeTipLift::canInfoCallback, this);

    vehicle_cmd_sub_ = nh.subscribe<autoware_msgs::VehicleCmd>("/vehicle_cmd_smooth", 10, 
                    &SafeTipLift::vehicleCmdCallback, this);

    arrived_flag_sub_ = nh.subscribe<std_msgs::Int8>("/arrived_flag", 10, 
                        &SafeTipLift::arrivedFlagCallback, this);

    feedback_status_sub_ = nh.subscribe<autoware_remove_msgs::State>("/feedback_status", 10,
                        &SafeTipLift::feedbackStatusCallback, this);

    safetiplift_pub_ = nh.advertise<std_msgs::UInt16MultiArray>("/safe_tip_lift", 10);
    
    // 初始化状态
    arrived_flag_ = 1;
    current_angle_ = 0.0;
    actual_lift_ = 0;
    current_speed_ = 0;
    state_ptr_ = nullptr;
    
    ROS_INFO("SafeTipLift initialized with:");
    ROS_INFO("  base_def_safelift: %d, base_pallet_safelift: %d", 
                base_def_safelift_, base_pallet_safelift_);
    ROS_INFO("  max_angle: %.3f rad, adjustment range: [%.1f, %.1f]", 
                max_angle_, min_adjustment_, max_adjustment_);
}

SafeTipLift::~SafeTipLift() {}

// CAN信息回调函数
void SafeTipLift::canInfoCallback(const autoware_can_msgs::CANInfo::ConstPtr& msg) {
    // current_angle_ = msg->angle;
    // publishSafeTipLift();
}

void SafeTipLift::vehicleCmdCallback(const autoware_msgs::VehicleCmd::ConstPtr& msg)
{
    current_angle_ = msg->ctrl_cmd.steering_angle;
    publishSafeTipLift();
}

// 到达标志回调函数
void SafeTipLift::arrivedFlagCallback(const std_msgs::Int8::ConstPtr& msg) {
    arrived_flag_ = msg->data;
    ROS_DEBUG("Arrived flag updated to: %d", arrived_flag_);
    publishSafeTipLift();
}

// 反馈状态回调函数
void SafeTipLift::feedbackStatusCallback(const autoware_remove_msgs::State::ConstPtr& msg) {
    // 保存State消息指针
    state_ptr_ = msg;
    
    // 更新内部字段
    actual_lift_ = msg->VehicleInfo.lift;
    inplace_ = msg->VehicleInfo.inplace;
    current_speed_ = msg->VehicleCmd.speed;
    ROS_DEBUG("Actual lift distance updated to: %d", actual_lift_);
    publishSafeTipLift();
}

// 发布安全叉尖距离
void SafeTipLift::publishSafeTipLift() {
    std_msgs::UInt16MultiArray safelift_msg;
    
    // 根据arrived_flag选择基础安全距离
    uint16_t base_safelift = (arrived_flag_ == 1) ? base_pallet_safelift_ : base_def_safelift_;

    // 根据角度动态调整安全距离（角度越大，安全距离越小）
    uint16_t adjusted_safelift = calculateInverseSafeLift(base_safelift, current_angle_);


    if (adjusted_safelift < 15)
        adjusted_safelift = 15;
    
    // 判断是否低于阈值（0或1）
    uint16_t below_threshold = (actual_lift_ < adjusted_safelift && (state_ptr_!=nullptr && state_ptr_->TaskInfo.state != 0)) ? 1 : 0;
    
    if (state_ptr_!=nullptr &&
        state_ptr_->VehicleInfo.param[0] < 0 &&     // 倒车
        state_ptr_->TaskInfo.type == 1 &&      // 任务类型为装卸
        state_ptr_->TaskInfo.state != 0 &&      // 任务状态为进行中
        state_ptr_->TaskInfo.site.dis < 0.05)
    {
        below_threshold = 0;
    }

    // 设置数组大小为3
    safelift_msg.data.resize(3);
    safelift_msg.data[0] = adjusted_safelift;  // 叉尖阈值
    safelift_msg.data[1] = actual_lift_;       // 叉尖实际距离
    safelift_msg.data[2] = below_threshold;    // 是否低于阈值（0或1）
    
    safetiplift_pub_.publish(safelift_msg);
    
    ROS_INFO("SafeTipLift: flag=%d, angle=%.3f rad, threshold=%d, actual=%d, below_threshold=%d", 
                        arrived_flag_, current_angle_, adjusted_safelift, actual_lift_, below_threshold);
}

// 使用反向平滑函数计算安全距离（角度越大，安全距离越小）
uint16_t SafeTipLift::calculateInverseSafeLift(uint16_t base_safelift, double angle_rad) {
    // 角度绝对值，用于计算调整系数
    double abs_angle_rad = std::abs(angle_rad);
    
    // 限制角度在合理范围内
    double clamped_angle = std::min(abs_angle_rad, max_angle_);
    
    // 计算归一化的角度比例 (0到1之间)
    double normalized_angle = clamped_angle / max_angle_;
    
    // 使用反向平滑函数计算调整系数
    // 角度越大，调整系数越小，安全距离越小
    double adjustment_factor = max_adjustment_ - 
                                (max_adjustment_ - min_adjustment_) * 
                                std::pow(normalized_angle, smoothness_factor_);
    
    // 计算调整后的值
    uint16_t adjusted_value = static_cast<uint16_t>(base_safelift * adjustment_factor);
    
    // 确保不会小于最小安全距离
    uint16_t min_safe_value = static_cast<uint16_t>(base_safelift * min_adjustment_);
    return std::max(adjusted_value, min_safe_value);
}

// 备选方案：使用反向S形曲线实现更平滑的过渡
uint16_t SafeTipLift::calculateInverseSigmoidSafeLift(uint16_t base_safelift, double angle_rad) {
    // 角度绝对值，用于计算调整系数
    double abs_angle_rad = std::abs(angle_rad);
    
    // 限制角度在合理范围内
    double clamped_angle = std::min(abs_angle_rad, max_angle_);
    
    // 计算归一化的角度比例 (0到1之间)
    double normalized_angle = clamped_angle / max_angle_;
    
    // 使用反向S形函数计算调整系数
    // 将角度映射到S形函数的反向
    double sigmoid = 1.0 / (1.0 + std::exp(10.0 * (normalized_angle - 0.5)));
    
    // 将S形函数输出映射到调整系数范围
    double adjustment_factor = min_adjustment_ + 
                                (max_adjustment_ - min_adjustment_) * sigmoid;
    
    // 计算调整后的值
    uint16_t adjusted_value = static_cast<uint16_t>(base_safelift * adjustment_factor);
    
    // 确保不会小于最小安全距离
    uint16_t min_safe_value = static_cast<uint16_t>(base_safelift * min_adjustment_);
    return std::max(adjusted_value, min_safe_value);
}

// 备选方案：使用倒数函数实现角度越大安全距离越小
uint16_t SafeTipLift::calculateReciprocalSafeLift(uint16_t base_safelift, double angle_rad) {
    // 角度绝对值，用于计算调整系数
    double abs_angle_rad = std::abs(angle_rad);
    
    // 使用倒数函数计算调整系数
    // 当角度为0时，调整系数为max_adjustment_
    // 当角度增加时，调整系数逐渐减小
    double k = 0.5; // 控制曲线陡峭程度的参数
    double adjustment_factor = max_adjustment_ / (1.0 + k * abs_angle_rad);
    
    // 确保调整系数不低于最小值
    adjustment_factor = std::max(adjustment_factor, min_adjustment_);
    
    // 计算调整后的值
    uint16_t adjusted_value = static_cast<uint16_t>(base_safelift * adjustment_factor);
    
    return adjusted_value;
}