# Safe Obstacle Detection Node

`safe_obstacle` 是一个高性能、高可靠性的自动驾驶碰撞感知 ROS 节点。它通过激光雷达点云在车辆周围建立动态变化的**多边形安全区域**（紧急制动区和减速区），实时检测障碍物入侵并输出安全等级信号。

![ROS](https://img.shields.io/badge/ROS-noetic%2Fmelodic-blue) ![PCL](https://img.shields.io/badge/PCL-1.10+-green) ![C++](https://img.shields.io/badge/C++-11-orange)

## 🌟 主要特性

*   **动态缩放逻辑**：根据车辆当前速度（CAN 反馈）实时伸缩检测多边形范围。采用指数衰减函数平滑处理高速情况，确保探测距离随速度增加。
*   **双模式自动切换**：支持“常规模式”与“窄道模式”。通过监控路径行为 ID（Behavior ID 3 或 4）自动收窄检测框，解决窄路误触发问题。
*   **前后双向检测**：根据行驶方向（前进/倒车）自动切换对应的多边形检测集。
*   **高性能优化**：
    *   **AABB 预过滤**：在执行复杂的射线检测前，先通过外接矩形（Bounding Box）快速筛选点云。
    *   **Early Exit 逻辑**：一旦检测到紧急碰撞点立即停止当前帧计算，极大降低 CPU 占用。
    *   **滑动窗口滤波**：多帧点云融合，有效减少噪点引起的误触发。
*   **极高健壮性**：彻底重构了参数加载逻辑，增加判空保护与异常拦截，防止因 YAML 参数缺失导致的节点崩溃或死锁（Communication Failed）。

## 🛠 依赖项

*   **ROS**: Melodic / Noetic
*   **PCL**: 1.10 或更高版本
*   **核心包**: `pcl_ros`, `std_msgs`, `geometry_msgs`
*   **Autoware 消息定义**:
    *   `autoware_msgs`
    *   `autoware_can_msgs`

## 🚀 安装与编译

```bash
# 进入工作空间
cd ~/catkin_ws/src

# 克隆代码
git clone https://github.com/your-repo/safe_obstacle.git

# 编译
cd ..
catkin_make
source devel/setup.bash
```

## 📡 话题接口

### 订阅 (Subscribed)
| 话题名称 | 消息类型 | 说明 |
| :--- | :--- | :--- |
| `/points_filter` | `sensor_msgs/PointCloud2` | 过滤后的激光雷达点云输入 |
| `/can_info` | `autoware_can_msgs/CANInfo` | 提供车辆实时速度用于动态缩放 |
| `/twist_raw` | `geometry_msgs/TwistStamped` | 用于判断行驶方向（前进/后退） |
| `/lqr_targetwayp`| `autoware_msgs/Waypoint` | 监控路段属性，触发窄道模式切换 |

### 发布 (Published)
| 话题名称 | 消息类型 | 说明 |
| :--- | :--- | :--- |
| `/safe_obstacle_data` | `std_msgs/Int8` | **0**:安全；**1**:紧急制动；**2**:减速 |
| `/safe_obstacle_markers` | `visualization_msgs/MarkerArray` | Rviz 可视化多边形检测框 |

## ⚙️ 参数配置 (YAML)

配置文件路径：`$(env HOME)/work/workspace/config/perception/safe_obstacle.yaml`

### 核心参数说明
| 参数 | 说明 |
| :--- | :--- |
| `debug_mode` | 开启后会在终端输出详细的运行日志、点数监控及模式切换提醒 |
| `max_longitudinal_scale` | 纵向最大缩放比例（高速时） |
| `min_longitudinal_scale` | 纵向最小缩放比例（静止或极低速时） |
| `reference_speed` | 基准速度 (m/s)，速度达到此值后缩放比例为 1.0 |
| `exigencyrect_1` | 常规模式下的车辆紧急避障多边形顶点坐标 |
| `exigencyrect_2` | 窄道模式下的紧急避障多边形顶点坐标 |

## 🔍 调试与可视化

1.  **启动节点**：
    ```bash
    roslaunch safe_obstacle safe_obstacle.launch
    ```
2.  **Rviz 可视化**：
    *   添加 `MarkerArray` 插件。
    *   订阅 `/safe_obstacle_markers`。
    *   `Fixed Frame` 请确保设置为 `velodyne`（或 YAML 中定义的 Frame）。
3.  **日志监控**：
    当 `debug_mode: true` 时，您可以通过终端实时查看节点状态：
    ```text
    [INFO] [SafeObstacle] Node active. Processing cloud (Points: 24500)
    [INFO] [SafeObstacle] Switch to NARROW mode
    ```

## ⚠️ 注意事项

*   **路径依赖**：Launch 文件默认通过环境变量 `$(env HOME)` 定位配置文件，请确保目录结构符合 `/home/user/work/workspace/...`。
*   **坐标系**：多边形顶点坐标是相对于激光雷达（velodyne）坐标系的，请确保点云坐标系一致。
