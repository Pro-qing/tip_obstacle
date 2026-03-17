#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include "tip_obstacle/tip_obstacle.h"
#include <tip_obstacle/tip_obstacleConfig.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>

using namespace sensor_msgs;
using namespace message_filters;

laser_geometry::LaserProjection projector_;
ros::Publisher point_cloud_publisher_;
ros::Publisher fused_point_cloud_publisher_;
ros::Publisher point_mindis;

std::string parent_frame_;
std::string child_frame_;

ros::Subscriber pallet_id_sub_, tip_sub_;


int tip_type_;
int pallet_id_state_;

double left_x = 0;
double left_y = 0;
double left_z = 0;
double left_yaw = 0;
double left_pitch = 0;
double left_roll = 0;

double right_x = 0;
double right_y = 0;
double right_z = 0;
double right_yaw = 0;
double right_pitch = 0;
double right_roll = 0;


double fused_x = 0;
double fused_y = 0;
double fused_z = 0;
double fused_yaw = 0;
double fused_pitch = 0;
double fused_roll = 0;


int left_filter_enable;
int right_filter_enable;

double left_min_angle;
double left_max_angle;
double right_min_angle;
double right_max_angle;

double left_min_y;
double left_max_y;
double right_min_y;
double right_max_y;

int pallet_id_left_filter_enable;
int pallet_id_right_filter_enable;

double pallet_id_left_min_angle;
double pallet_id_left_max_angle;
double pallet_id_right_min_angle;
double pallet_id_right_max_angle;

double pallet_id_left_min_y;
double pallet_id_left_max_y;
double pallet_id_right_min_y;
double pallet_id_right_max_y;

void palletIdCallback(const std_msgs::Int8::ConstPtr &msg)
{
    pallet_id_state_ = msg->data;
}


//两雷达
void callback(const sensor_msgs::LaserScan::ConstPtr &msg1,
              const sensor_msgs::LaserScan::ConstPtr &msg2) 
{
    ROS_ERROR("AAAA");

    sensor_msgs::PointCloud2 left_msgs, right_msgs;
    projector_.projectLaser(*msg1, left_msgs);
    projector_.projectLaser(*msg2, right_msgs);

    std_msgs::Float32 min_dis;
    min_dis.data = 255.0;


    pcl::PointCloud<pcl::PointXYZI>::Ptr left_cloudPoints(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(left_msgs, *left_cloudPoints);
    if (pallet_id_state_) {
        if (pallet_id_left_filter_enable)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr filter_laser(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointXYZI point_;
            for (std::size_t i = 0; i < left_cloudPoints->size(); ++i)
            {
                double angle = atan2(left_cloudPoints->points[i].y, left_cloudPoints->points[i].x);
                angle = (int)(angle*180./3.14159 + 360) % 360;
                if(pallet_id_left_min_angle > pallet_id_left_max_angle){
                    if (angle > pallet_id_left_min_angle || angle < pallet_id_left_max_angle)
                        continue;
                }
                else{
                    if (angle > pallet_id_left_min_angle && angle < pallet_id_left_max_angle)
                        continue;
                }
                // y轴过滤
                double y_coord = left_cloudPoints->points[i].y;
                if(pallet_id_left_min_y > pallet_id_left_max_y){
                    if (y_coord > pallet_id_left_min_y || y_coord < pallet_id_left_max_y)
                        continue;
                }
                else{
                    if (y_coord > pallet_id_left_min_y && y_coord < pallet_id_left_max_y)
                        continue;
                }
                float radius = sqrt(pow(left_cloudPoints->points[i].x,2)+pow(left_cloudPoints->points[i].y,2));  //算出每个点的半径
                if(radius < fabs(min_dis.data))
                    min_dis.data = radius;
                filter_laser->points.push_back(left_cloudPoints->points[i]);
            }
            left_cloudPoints = filter_laser;
        }
    }
    else{
        if (left_filter_enable)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr filter_laser(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointXYZI point_;
            for (std::size_t i = 0; i < left_cloudPoints->size(); ++i)
            {
                double angle = atan2(left_cloudPoints->points[i].y, left_cloudPoints->points[i].x);
                angle = (int)(angle*180./3.14159 + 360) % 360;
                if(left_min_angle > left_max_angle){
                    if (angle > left_min_angle || angle < left_max_angle)
                        continue;
                }
                else{
                    if (angle > left_min_angle && angle < left_max_angle)
                        continue;
                }
                // y轴过滤
                double y_coord = left_cloudPoints->points[i].y;
                if(left_min_y > left_max_y){
                    if (y_coord > left_min_y || y_coord < left_max_y)
                        continue;
                }
                else{
                    if (y_coord > left_min_y && y_coord < left_max_y)
                        continue;
                }
                float radius = sqrt(pow(left_cloudPoints->points[i].x,2)+pow(left_cloudPoints->points[i].y,2));  //算出每个点的半径
                if(radius < fabs(min_dis.data))
                    min_dis.data = radius;
                filter_laser->points.push_back(left_cloudPoints->points[i]);
            }
            left_cloudPoints = filter_laser;
        }
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr right_cloudPoints(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(right_msgs, *right_cloudPoints);
    if (pallet_id_state_) {
        if (pallet_id_right_filter_enable)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr filter_laser(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointXYZI point_;
            for (std::size_t i = 0; i < right_cloudPoints->size(); ++i)
            {
                double angle = atan2(right_cloudPoints->points[i].y, right_cloudPoints->points[i].x);
                angle = (int)(angle*180./3.14159 + 360) % 360;
                if(pallet_id_right_min_angle > pallet_id_right_max_angle){
                    if (angle > pallet_id_right_min_angle || angle < pallet_id_right_max_angle)
                        continue;
                }
                else{
                    if (angle > pallet_id_right_min_angle && angle < pallet_id_right_max_angle)
                        continue;
                }
                // y轴过滤
                double y_coord = right_cloudPoints->points[i].y;
                if(pallet_id_right_min_y > pallet_id_right_max_y){
                    if (y_coord > pallet_id_right_min_y || y_coord < pallet_id_right_max_y)
                        continue;
                }
                else{
                    if (y_coord > pallet_id_right_min_y && y_coord < pallet_id_right_max_y)
                        continue;
                }
                float radius = sqrt(pow(right_cloudPoints->points[i].x,2)+pow(right_cloudPoints->points[i].y,2));  //算出每个点的半径
                if(radius < fabs(min_dis.data))
                    min_dis.data = radius;
                filter_laser->points.push_back(right_cloudPoints->points[i]);
            }
            right_cloudPoints = filter_laser;
        }
    }else{
         if (right_filter_enable)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr filter_laser(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointXYZI point_;
            for (std::size_t i = 0; i < right_cloudPoints->size(); ++i)
            {
                double angle = atan2(right_cloudPoints->points[i].y, right_cloudPoints->points[i].x);
                angle = (int)(angle*180./3.14159 + 360) % 360;
                if(right_min_angle > right_max_angle){
                    if (angle > right_min_angle || angle < right_max_angle)
                        continue;
                }
                else{
                    if (angle > right_min_angle && angle < right_max_angle)
                        continue;
                }
                // y轴过滤
                double y_coord = right_cloudPoints->points[i].y;
                if(right_min_y > right_max_y){
                    if (y_coord > right_min_y || y_coord < right_max_y)
                        continue;
                }
                else{
                    if (y_coord > right_min_y && y_coord < right_max_y)
                        continue;
                }
                float radius = sqrt(pow(right_cloudPoints->points[i].x,2)+pow(right_cloudPoints->points[i].y,2));  //算出每个点的半径
                if(radius < fabs(min_dis.data))
                    min_dis.data = radius;
                filter_laser->points.push_back(right_cloudPoints->points[i]);
            }
            right_cloudPoints = filter_laser;
        }
    }

    Eigen::Affine3f left_calibration_matrix = Eigen::Affine3f::Identity();
    // 在 X 轴上定义一个 2.5 米的平移.
    left_calibration_matrix.translation() << left_x, left_y, left_z;
    // 和前面一样的旋转; Z 轴上旋转 theta 弧度
    left_calibration_matrix.rotate(Eigen::AngleAxisf(left_yaw, Eigen::Vector3f::UnitZ()));
    left_calibration_matrix.rotate(Eigen::AngleAxisf(left_pitch, Eigen::Vector3f::UnitY()));
    left_calibration_matrix.rotate(Eigen::AngleAxisf(left_roll, Eigen::Vector3f::UnitX()));
    pcl::transformPointCloud(*left_cloudPoints, *left_cloudPoints, left_calibration_matrix);

    Eigen::Affine3f right_calibration_matrix = Eigen::Affine3f::Identity();
    // 在 X 轴上定义一个 2.5 米的平移.
    right_calibration_matrix.translation() << right_x, right_y, right_z;
    // 和前面一样的旋转; Z 轴上旋转 theta 弧度
    right_calibration_matrix.rotate(Eigen::AngleAxisf(right_yaw, Eigen::Vector3f::UnitZ()));
    right_calibration_matrix.rotate(Eigen::AngleAxisf(right_pitch, Eigen::Vector3f::UnitY()));
    right_calibration_matrix.rotate(Eigen::AngleAxisf(right_roll, Eigen::Vector3f::UnitX()));
    pcl::transformPointCloud(*right_cloudPoints, *right_cloudPoints, right_calibration_matrix);

    *left_cloudPoints = *left_cloudPoints + *right_cloudPoints;



    sensor_msgs::PointCloud2 outMsg;
    static int count = 0;
    pcl::toROSMsg(*left_cloudPoints, outMsg);
    outMsg.header.frame_id = child_frame_;
    outMsg.header.seq = ++count;
    outMsg.header.stamp = ros::Time::now();
    point_cloud_publisher_.publish(outMsg);


    Eigen::Affine3f fused_calibration_matrix = Eigen::Affine3f::Identity();
    // 在 X 轴上定义一个 2.5 米的平移.
    fused_calibration_matrix.translation() << fused_x, fused_y, fused_z;
    // 和前面一样的旋转; Z 轴上旋转 theta 弧度
    fused_calibration_matrix.rotate(Eigen::AngleAxisf(fused_yaw, Eigen::Vector3f::UnitZ()));
    fused_calibration_matrix.rotate(Eigen::AngleAxisf(fused_pitch, Eigen::Vector3f::UnitY()));
    fused_calibration_matrix.rotate(Eigen::AngleAxisf(fused_roll, Eigen::Vector3f::UnitX()));
    pcl::transformPointCloud(*left_cloudPoints, *left_cloudPoints, fused_calibration_matrix);


    sensor_msgs::PointCloud2 fused_outMsg;
    static int fused_count = 0;
    pcl::toROSMsg(*left_cloudPoints, fused_outMsg);
    fused_outMsg.header.frame_id = parent_frame_;
    fused_outMsg.header.seq = ++fused_count;
    fused_outMsg.header.stamp = ros::Time::now();
    fused_point_cloud_publisher_.publish(fused_outMsg);
 
    point_mindis.publish(min_dis);
    // ROS_INFO("min_dis:  %f", min_dis.data);
}


void callback_swb(const sensor_msgs::LaserScan::ConstPtr &msg) 
{
    sensor_msgs::PointCloud2 left_msgs, right_msgs;
    projector_.projectLaser(*msg, left_msgs);

    std_msgs::Float32 min_dis;
    min_dis.data = 255.0;



    pcl::PointCloud<pcl::PointXYZI>::Ptr left_cloudPoints(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(left_msgs, *left_cloudPoints);
    if (pallet_id_state_) {
        if (pallet_id_left_filter_enable)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr filter_laser(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointXYZI point_;
            for (std::size_t i = 0; i < left_cloudPoints->size(); ++i)
            {
                double angle = atan2(left_cloudPoints->points[i].y, left_cloudPoints->points[i].x);
                angle = (int)(angle*180./3.14159 + 360) % 360;
                if(pallet_id_left_min_angle > pallet_id_left_max_angle){
                    if (angle > pallet_id_left_min_angle || angle < pallet_id_left_max_angle)
                        continue;
                }
                else{
                    if (angle > pallet_id_left_min_angle && angle < pallet_id_left_max_angle)
                        continue;
                }
                // y轴过滤
                double y_coord = left_cloudPoints->points[i].y;
                if(pallet_id_left_min_y > pallet_id_left_max_y){
                    if (y_coord > pallet_id_left_min_y || y_coord < pallet_id_left_max_y)
                        continue;
                }
                else{
                    if (y_coord > pallet_id_left_min_y && y_coord < pallet_id_left_max_y)
                        continue;
                }
                float radius = sqrt(pow(left_cloudPoints->points[i].x,2)+pow(left_cloudPoints->points[i].y,2));  //算出每个点的半径
                if(radius < fabs(min_dis.data))
                    min_dis.data = radius;
                filter_laser->points.push_back(left_cloudPoints->points[i]);
            }
            left_cloudPoints = filter_laser;
        }
    }
    else{
        if (left_filter_enable)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr filter_laser(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointXYZI point_;
            for (std::size_t i = 0; i < left_cloudPoints->size(); ++i)
            {
                double angle = atan2(left_cloudPoints->points[i].y, left_cloudPoints->points[i].x);
                angle = (int)(angle*180./3.14159 + 360) % 360;
                if(left_min_angle > left_max_angle){
                    if (angle > left_min_angle || angle < left_max_angle)
                        continue;
                }
                else{
                    if (angle > left_min_angle && angle < left_max_angle)
                        continue;
                }
                // y轴过滤
                double y_coord = left_cloudPoints->points[i].y;
                if(left_min_y > left_max_y){
                    if (y_coord > left_min_y || y_coord < left_max_y)
                        continue;
                }
                else{
                    if (y_coord > left_min_y && y_coord < left_max_y)
                        continue;
                }
                float radius = sqrt(pow(left_cloudPoints->points[i].x,2)+pow(left_cloudPoints->points[i].y,2));  //算出每个点的半径
                if(radius < fabs(min_dis.data))
                    min_dis.data = radius;
                filter_laser->points.push_back(left_cloudPoints->points[i]);
            }
            left_cloudPoints = filter_laser;
        }
    }


    Eigen::Affine3f left_calibration_matrix = Eigen::Affine3f::Identity();
    // 在 X 轴上定义一个 2.5 米的平移.
    left_calibration_matrix.translation() << left_x, left_y, left_z;
    // 和前面一样的旋转; Z 轴上旋转 theta 弧度
    left_calibration_matrix.rotate(Eigen::AngleAxisf(left_yaw, Eigen::Vector3f::UnitZ()));
    left_calibration_matrix.rotate(Eigen::AngleAxisf(left_pitch, Eigen::Vector3f::UnitY()));
    left_calibration_matrix.rotate(Eigen::AngleAxisf(left_roll, Eigen::Vector3f::UnitX()));
    pcl::transformPointCloud(*left_cloudPoints, *left_cloudPoints, left_calibration_matrix);

    sensor_msgs::PointCloud2 outMsg;
    static int count = 0;
    pcl::toROSMsg(*left_cloudPoints, outMsg);
    outMsg.header.frame_id = child_frame_;
    outMsg.header.seq = ++count;
    outMsg.header.stamp = ros::Time::now();
    point_cloud_publisher_.publish(outMsg);


    Eigen::Affine3f fused_calibration_matrix = Eigen::Affine3f::Identity();
    // 在 X 轴上定义一个 2.5 米的平移.
    fused_calibration_matrix.translation() << fused_x, fused_y, fused_z;
    // 和前面一样的旋转; Z 轴上旋转 theta 弧度
    fused_calibration_matrix.rotate(Eigen::AngleAxisf(fused_yaw, Eigen::Vector3f::UnitZ()));
    fused_calibration_matrix.rotate(Eigen::AngleAxisf(fused_pitch, Eigen::Vector3f::UnitY()));
    fused_calibration_matrix.rotate(Eigen::AngleAxisf(fused_roll, Eigen::Vector3f::UnitX()));
    pcl::transformPointCloud(*left_cloudPoints, *left_cloudPoints, fused_calibration_matrix);


    sensor_msgs::PointCloud2 fused_outMsg;
    static int fused_count = 0;
    pcl::toROSMsg(*left_cloudPoints, fused_outMsg);
    fused_outMsg.header.frame_id = parent_frame_;
    fused_outMsg.header.seq = ++fused_count;
    fused_outMsg.header.stamp = ros::Time::now();
    fused_point_cloud_publisher_.publish(fused_outMsg);
 
    point_mindis.publish(min_dis);
    // ROS_INFO("min_dis:  %f", min_dis.data);
}



//获取动态参数回调服务函数
void param_callback(tip_obstacle::tip_obstacleConfig &config,uint32_t level)
{
    left_x = config.left_x;
    left_y = config.left_y;
    left_z = config.left_z;
    left_yaw = config.left_yaw;
    left_pitch = config.left_pitch;
    left_roll = config.left_roll;

    right_x = config.right_x;
    right_y = config.right_y;
    right_z = config.right_z;
    right_yaw = config.right_yaw;
    right_pitch = config.right_pitch;
    right_roll = config.right_roll;

    fused_x = config.fused_x;
    fused_y = config.fused_y;
    fused_z = config.fused_z;
    fused_yaw = config.fused_yaw;
    fused_pitch = config.fused_pitch;
    fused_roll = config.fused_roll;

    left_min_angle = config.left_min_angle;
    left_max_angle = config.left_max_angle;
    right_min_angle = config.right_min_angle;
    right_max_angle = config.right_max_angle;

    left_min_y = config.left_min_y;
    left_max_y = config.left_max_y;
    right_min_y = config.right_min_y;
    right_max_y = config.right_max_y;

    pallet_id_left_min_y = config.pallet_id_left_min_y;
    pallet_id_left_max_y = config.pallet_id_left_max_y;
    pallet_id_right_min_y = config.pallet_id_right_min_y;
    pallet_id_right_max_y = config.pallet_id_right_max_y;

    ROS_WARN("%f  %f  %f  %f  %f  %f", left_x, left_y, left_z, left_yaw, left_pitch, left_roll);
    ROS_WARN("%f  %f  %f  %f  %f  %f", right_x, right_y, right_z, right_yaw, right_pitch, right_roll);
    ROS_WARN("%f  %f  %f  %f  %f  %f", fused_x, fused_y, fused_z, fused_yaw, fused_pitch, fused_roll);

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tip_obstacle");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    std::string points_parent_src;
    std::string points_child_one;
    std::string out_points_cloud;
    std::string out_fused_points_cloud;

    pallet_id_state_ = -1;

    //定义一个动态参数配置服务
    dynamic_reconfigure::Server<tip_obstacle::tip_obstacleConfig> server;
    //定义一个动态参数配置服务的回调type
    dynamic_reconfigure::Server<tip_obstacle::tip_obstacleConfig>::CallbackType f;
    //绑定callback函数
    f = boost::bind(&param_callback,_1,_2);
    //设置callback函数
    server.setCallback(f);

    p_nh.param<int>("tip_type", tip_type_, 0);
    p_nh.param<std::string>("parent_frame", parent_frame_, "laser");
    p_nh.param<std::string>("child_frame", child_frame_, "laser");
    
    p_nh.param<std::string>("out_points_cloud", out_points_cloud, "points_tip");
    p_nh.param<std::string>("out_fused_points_cloud", out_fused_points_cloud, "fused_points_tip");


    p_nh.param<double>("left_x", left_x, 0);
    p_nh.param<double>("left_y", left_y, 0);
    p_nh.param<double>("left_z", left_z, 0);
    p_nh.param<double>("left_yaw", left_yaw, 0);
    p_nh.param<double>("left_pitch", left_pitch, 0);
    p_nh.param<double>("left_roll", left_roll, 0);

    p_nh.param<double>("right_x", right_x, 0);
    p_nh.param<double>("right_y", right_y, 0);
    p_nh.param<double>("right_z", right_z, 0);
    p_nh.param<double>("right_yaw", right_yaw, 0);
    p_nh.param<double>("right_pitch", right_pitch, 0);
    p_nh.param<double>("right_roll", right_roll, 0);

    p_nh.param<double>("fused_x", fused_x, 0);
    p_nh.param<double>("fused_y", fused_y, 0);
    p_nh.param<double>("fused_z", fused_z, 0);
    p_nh.param<double>("fused_yaw", fused_yaw, 0);
    p_nh.param<double>("fused_pitch", fused_pitch, 0);
    p_nh.param<double>("fused_roll", fused_roll, 0);


    p_nh.param<int>("left_filter_enable", left_filter_enable, 0);
    p_nh.param<double>("left_min_angle", left_min_angle, 0.0);
    p_nh.param<double>("left_max_angle", left_max_angle, 0.0);

    p_nh.param<int>("right_filter_enable", right_filter_enable, 0);
    p_nh.param<double>("right_min_angle", right_min_angle, 0.0);
    p_nh.param<double>("right_max_angle", right_max_angle, 0.0);

    p_nh.param<double>("left_min_y", left_min_y, -10.0);
    p_nh.param<double>("left_max_y", left_max_y, 10.0);
    p_nh.param<double>("right_min_y", right_min_y, -10.0);
    p_nh.param<double>("right_max_y", right_max_y, 10.0);

    p_nh.param<int>("pallet_id_left_filter_enable", pallet_id_left_filter_enable, 0);
    p_nh.param<double>("pallet_id_left_min_angle", pallet_id_left_min_angle, 0.0);
    p_nh.param<double>("pallet_id_left_max_angle", pallet_id_left_max_angle, 0.0);

    p_nh.param<int>("pallet_id_right_filter_enable", pallet_id_right_filter_enable, 0);
    p_nh.param<double>("pallet_id_right_min_angle", pallet_id_right_min_angle, 0.0);
    p_nh.param<double>("pallet_id_right_max_angle", pallet_id_right_max_angle, 0.0);

    p_nh.param<double>("pallet_id_left_min_y", pallet_id_left_min_y, -10.0);
    p_nh.param<double>("pallet_id_left_max_y", pallet_id_left_max_y, 10.0);
    p_nh.param<double>("pallet_id_right_min_y", pallet_id_right_min_y, -10.0);
    p_nh.param<double>("pallet_id_right_max_y", pallet_id_right_max_y, 10.0);



    point_cloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(out_points_cloud, 10);
    fused_point_cloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(out_fused_points_cloud, 10);
    point_mindis = nh.advertise<std_msgs::Float32>("tip_dis", 10);
    
    pallet_id_sub_ = nh.subscribe("/arrived_flag", 10, &palletIdCallback);
    ROS_ERROR("%d",tip_type_);
    //①分别接收两个topic
    message_filters::Subscriber<sensor_msgs::LaserScan> sub_scan1(nh, "/scan_bleft", 1, ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<sensor_msgs::LaserScan> sub_scan2(nh, "/scan_bright", 1, ros::TransportHints().tcpNoDelay());

    // ②将两个topic的数据进行同步
    typedef sync_policies::ApproximateTime<sensor_msgs::LaserScan,
                                            sensor_msgs::LaserScan>
        syncPolicy;
    Synchronizer<syncPolicy> sync(syncPolicy(10), sub_scan1, sub_scan2);
    if (tip_type_ == 0) // 
    {
        
        sync.registerCallback(boost::bind(&callback, _1, _2));
    }
    else if (tip_type_ == 1)
    {
        tip_sub_ = nh.subscribe("/scan_bleft", 10, &callback_swb);
    }





    tf::TransformBroadcaster broadcaster;
    ros::Rate rate(10);
    
    while (ros::ok()) {
        ros::spinOnce();
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(fused_x, fused_y, fused_z));
        tf::Quaternion q;
        q.setRPY(fused_roll, fused_pitch, fused_yaw);
        transform.setRotation(q);
        
        broadcaster.sendTransform(
            tf::StampedTransform(transform, ros::Time::now(), parent_frame_, "tip_fused_laser"));
        
        rate.sleep();
    }


    return 0;
}
