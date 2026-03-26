cmake_minimum_required(VERSION 3.0.2)
project(tip_obstacle)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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
)

# 单独引入系统库 yaml-cpp 
find_package(yaml-cpp REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/tip_obstacle.cfg
)


# find_package(PCL 1.2 REQUIRED)
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})
# list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imu_conversion
#  CATKIN_DEPENDS pcl_conversions pcl_msgs pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include 
  ${catkin_INCLUDE_DIRS}
)


add_executable(tip_obstacle_node src/tip_obstacle.cpp)


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(environment ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(tip_obstacle_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(environment
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
# )
add_dependencies(tip_obstacle_node ${PROJECT_NAME}_gencfg)
target_link_libraries(tip_obstacle_node
  ${catkin_LIBRARIES}
  yaml-cpp
  pthread
  # ${PCL_LIBRARIES}
)


#3 头文件,                   (PATH: install/include/slam_karto/)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#3 拷贝 launch目录           (PATH: install/share/slam_karto/launch/)
install(DIRECTORY 
    launch                  #注意,不能使用 launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS 
tip_obstacle_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
