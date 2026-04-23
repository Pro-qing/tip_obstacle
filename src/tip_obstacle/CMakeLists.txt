cmake_minimum_required(VERSION 3.0.2)
project(tip_obstacle)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  message_filters
  pcl_conversions
  pcl_msgs
  pcl_ros
  laser_geometry
  autoware_remove_msgs
  tf
  visualization_msgs
)

# 单独引入系统库 yaml-cpp 
find_package(yaml-cpp REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imu_conversion
#  CATKIN_DEPENDS pcl_conversions pcl_msgs pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
 include 
  ${catkin_INCLUDE_DIRS}
)

add_executable(tip_obstacle_node src/tip_obstacle.cpp)

# 确保在编译当前节点前，其他功能包的自定义消息头文件已经生成
add_dependencies(tip_obstacle_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 链接库 (已经去掉了 _gencfg 的依赖)
target_link_libraries(tip_obstacle_node
  ${catkin_LIBRARIES}
  yaml-cpp
  pthread
)

###########
## Install ##
###########

# 拷贝头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# 拷贝 launch 目录
install(DIRECTORY 
    launch                  
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 安装可执行文件
install(TARGETS 
  tip_obstacle_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)