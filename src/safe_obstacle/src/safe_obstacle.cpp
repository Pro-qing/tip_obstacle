#include "safe_obstacle.hpp"
#include <algorithm>

SafeObstacle::SafeObstacle(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh), pnh_(pnh)
{
    pnh_.param("debug_mode", debug_mode_, false);

    // 增强型参数解析 Lambda
    auto load_poly = [&](const std::string& key, std::vector<geometry_msgs::Point>& poly) {
        XmlRpc::XmlRpcValue xml_list;
        if (pnh_.getParam(key, xml_list) && xml_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            poly.clear();
            for (int i = 0; i < xml_list.size(); ++i) {
                geometry_msgs::Point p;
                auto read_val = [&](const std::string& field) -> double {
                    if (!xml_list[i].hasMember(field)) return 0.0;
                    XmlRpc::XmlRpcValue& v = xml_list[i][field];
                    if (v.getType() == XmlRpc::XmlRpcValue::TypeInt) return static_cast<double>(static_cast<int>(v));
                    if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble) return static_cast<double>(v);
                    return 0.0;
                };
                p.x = read_val("x");
                p.y = read_val("y");
                poly.push_back(p);
            }
            if(debug_mode_) ROS_INFO("[SafeObstacle] Loaded %s with %zu points", key.c_str(), poly.size());
            return true;
        }
        return false;
    };

    // 1. 加载所有两套多边形基准
    load_poly("exigencyrect_1", base_exigencySize_1);
    load_poly("exigencyrect_2", base_exigencySize_2);
    load_poly("slowrect_1", base_slowSize_1);
    load_poly("slowrect_2", base_slowSize_2);
    load_poly("reverse_exigencyrect_1", base_reverse_exigencySize_1);
    load_poly("reverse_exigencyrect_2", base_reverse_exigencySize_2);
    load_poly("reverse_slowrect_1", base_reverse_slowSize_1);
    load_poly("reverse_slowrect_2", base_reverse_slowSize_2);

    // 兜底逻辑：如果 _2 没配置，则使用 _1 的数据
    if (base_exigencySize_2.empty()) base_exigencySize_2 = base_exigencySize_1;
    if (base_slowSize_2.empty()) base_slowSize_2 = base_slowSize_1;
    if (base_reverse_exigencySize_2.empty()) base_reverse_exigencySize_2 = base_reverse_exigencySize_1;
    if (base_reverse_slowSize_2.empty()) base_reverse_slowSize_2 = base_reverse_slowSize_1;

    // 2. 加载动态缩放配置
    pnh_.param("max_longitudinal_scale", max_longitudinal_scale_, 1.5);
    pnh_.param("min_longitudinal_scale", min_longitudinal_scale_, 0.7);
    pnh_.param("max_lateral_scale", max_lateral_scale_, 1.1);
    pnh_.param("min_lateral_scale", min_lateral_scale_, 0.85);
    pnh_.param("longitudinal_sensitivity", longitudinal_sensitivity_, 0.8);
    pnh_.param("lateral_sensitivity", lateral_sensitivity_, 0.3);
    pnh_.param("reference_speed", reference_speed_, 3.0);

    // 3. 订阅与发布
    cloud_points_sub_ = nh_.subscribe("/points_filter", 1, &SafeObstacle::pointsCallback, this);
    twist_raw_sub_ = nh_.subscribe("/twist_raw", 1, &SafeObstacle::twist_raw_Callback, this);
    can_info_sub_ = nh_.subscribe("/can_info", 1, &SafeObstacle::can_info_Callback, this);
    lqr_targetwayp_sub_ = nh_.subscribe("/lqr_targetwayp", 1, &SafeObstacle::lqr_targetwayp_Callback, this);

    pub_markers = nh_.advertise<visualization_msgs::MarkerArray>("safe_obstacle_markers", 1);
    pub_safe = nh_.advertise<std_msgs::Int8>("safe_obstacle_data", 1);

    ROS_INFO("[SafeObstacle] Node Initialized.");
}

void SafeObstacle::lqr_targetwayp_Callback(const autoware_msgs::Waypoint::ConstPtr& msg) {
    if (!msg) return;
    auto& behaviors = msg->wpsattr.routeBehavior;
    // 判断是否进入窄道模式 (Behavior ID 3 或 4)
    bool is_narrow = std::any_of(behaviors.begin(), behaviors.end(), [](int b) { return b == 3 || b == 4; });
    if (is_narrow != is_narrow_channel_) {
        is_narrow_channel_ = is_narrow;
        if(debug_mode_) ROS_INFO("Mode Switched to: %s", is_narrow_channel_ ? "NARROW" : "NORMAL");
    }
}

void SafeObstacle::updateDetectionZones()
{
    // 根据当前模式选择对应的基础框
    const auto& b_ex = is_narrow_channel_ ? base_exigencySize_2 : base_exigencySize_1;
    const auto& b_sl = is_narrow_channel_ ? base_slowSize_2 : base_slowSize_1;
    const auto& b_rev_ex = is_narrow_channel_ ? base_reverse_exigencySize_2 : base_reverse_exigencySize_1;
    const auto& b_rev_sl = is_narrow_channel_ ? base_reverse_slowSize_2 : base_reverse_slowSize_1;

    double speed = can_info_ptr_ ? fabs(can_info_ptr_->speed) : 0.0;
    double ratio = speed / reference_speed_;

    auto calc_scale = [&](double min_s, double max_s, double sens) {
        if (ratio <= 1.0) return min_s + (1.0 - min_s) * ratio;
        return 1.0 + (max_s - 1.0) * (1.0 - exp(-sens * (ratio - 1.0)));
    };

    double lon_s = std::max(min_longitudinal_scale_, std::min(max_longitudinal_scale_, calc_scale(min_longitudinal_scale_, max_longitudinal_scale_, longitudinal_sensitivity_)));
    double lat_s = std::max(min_lateral_scale_, std::min(max_lateral_scale_, calc_scale(min_lateral_scale_, max_lateral_scale_, lateral_sensitivity_)));

    auto apply_scale = [&](const std::vector<geometry_msgs::Point>& in, std::vector<geometry_msgs::Point>& out, bool is_rev) {
        out.clear();
        for (const auto& p : in) {
            geometry_msgs::Point sp;
            // 只有朝向行驶方向的部分进行纵向缩放
            sp.x = (is_rev ? (p.x < 0 ? p.x * lon_s : p.x) : (p.x > 0 ? p.x * lon_s : p.x));
            sp.y = p.y * lat_s;
            out.push_back(sp);
        }
    };

    apply_scale(b_ex, current_exigencySize, false);
    apply_scale(b_sl, current_slowSize, false);
    apply_scale(b_rev_ex, current_reverse_exigencySize, true);
    apply_scale(b_rev_sl, current_reverse_slowSize, true);
}

void SafeObstacle::pointsCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    // 1. 无论有没有点云，首先更新并获取当前的检测区域
    updateDetectionZones();
    bool is_reverse = (twist_raw_ptr_ && (twist_raw_ptr_->header.frame_id == "back" || twist_raw_ptr_->twist.linear.x < -0.1));
    const auto& active_exigency = is_reverse ? current_reverse_exigencySize : current_exigencySize;
    const auto& active_slow = is_reverse ? current_reverse_slowSize : current_slowSize;

    // 2. 初始化默认输出状态为 0（安全）
    std_msgs::Int8 safeOut;
    safeOut.data = 0;

    // 3. 只有当点云非空时，才进行点云坐标转换和碰撞检测
    if (!msg->data.empty()) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *in_cloud_ptr);
        pcl::PointCloud<pcl::PointXYZI> filtered_cloud = SlidingWindow_Flitter(*in_cloud_ptr);

        for(const auto& p : filtered_cloud.points) {
            geometry_msgs::Point pt;
            pt.x = p.x; pt.y = p.y;
            
            if (PointInPolygon(pt, active_exigency)) {
                safeOut.data = 1;
                break; // 触发紧急停止，直接跳出
            }
            if (safeOut.data == 0 && PointInPolygon(pt, active_slow)) {
                safeOut.data = 2; // 触发减速，但继续遍历看是否会触发紧急停止
            }
        }
    }

    // 4. 无论如何，都发布可视化检测框和当前状态（这样下游就不会出现 no msg 了）
    visualization_msgs::MarkerArray markers_;
    // 内框颜色：触发红色，未触发橙色
    std_msgs::ColorRGBA col_ex = (safeOut.data == 1) ? createColor(1,0,0,1) : createColor(1,0.5,0,1);
    // 外框颜色：触发红色，未触发绿色
    std_msgs::ColorRGBA col_sl = (safeOut.data == 2) ? createColor(1,0,0,1) : createColor(0,1,0,1);

    markers_.markers.push_back(CreateMarker(active_exigency, col_ex, 0));
    markers_.markers.push_back(CreateMarker(active_slow, col_sl, 1));
    
    pub_markers.publish(markers_);
    pub_safe.publish(safeOut);
}

bool SafeObstacle::PointInPolygon(const geometry_msgs::Point& pt, const std::vector<geometry_msgs::Point>& poly)
{
    if (poly.size() < 3) return false;
    bool inside = false;
    for (int i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        if (((poly[i].y > pt.y) != (poly[j].y > pt.y)) &&
            (pt.x < (poly[j].x - poly[i].x) * (pt.y - poly[i].y) / (poly[j].y - poly[i].y) + poly[i].x)) {
            inside = !inside;
        }
    }
    return inside;
}

visualization_msgs::Marker SafeObstacle::CreateMarker(const std::vector<geometry_msgs::Point>& poly, std_msgs::ColorRGBA col, int id)
{
    visualization_msgs::Marker m;
    m.header.frame_id = "velodyne";
    m.header.stamp = ros::Time::now();
    m.ns = "safe_zones";
    m.id = id;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    
    if (poly.empty()) {
        m.action = visualization_msgs::Marker::DELETE;
        return m;
    }

    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = 0.05;
    m.color = col;
    m.pose.orientation.w = 1.0;
    for (const auto& p : poly) m.points.push_back(p);
    m.points.push_back(poly.front()); 
    return m;
}

pcl::PointCloud<pcl::PointXYZI> SafeObstacle::SlidingWindow_Flitter(pcl::PointCloud<pcl::PointXYZI> cloud) {
    static std::vector<pcl::PointCloud<pcl::PointXYZI>> buffer;
    buffer.push_back(cloud);
    if (buffer.size() > 3) buffer.erase(buffer.begin());
    pcl::PointCloud<pcl::PointXYZI> out;
    for (const auto& c : buffer) out += c;
    return out;
}

void SafeObstacle::twist_raw_Callback(const geometry_msgs::TwistStamped::Ptr &msg) { twist_raw_ptr_ = msg; }
void SafeObstacle::can_info_Callback(const autoware_can_msgs::CANInfo::Ptr &msg) { can_info_ptr_ = msg; }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "safe_obstacle_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    SafeObstacle obj(nh, pnh);
    ros::spin();
    return 0;
}