cmake_minimum_required(VERSION 3.0.2)
project(rm_match)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  image_transport
  message_generation
  cv_bridge
  nodelet
  roscpp
  rospy
  std_msgs
  tf
  tf2_geometry_msgs
  visualization_msgs
        pcl_conversions
        pcl_ros
        sensor_msgs
        dynamic_reconfigure
)


generate_dynamic_reconfigure_options(
        cfg/match.cfg
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rm_match
  CATKIN_DEPENDS image_transport message_generationcv_bridge nodelet roscpp rospy std_msgs tf tf2_geometry_msgs visualization_msgs
#  DEPENDS system_lib
)




include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)



add_library(${PROJECT_NAME} src/match.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
#add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
