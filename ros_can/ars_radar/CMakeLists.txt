cmake_minimum_required(VERSION 2.8.3)
project(ars_radar)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  jsk_recognition_msgs
  roscpp
  rospy
  std_msgs
  can_msgs
  pcl_ros
  sensor_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES usbcan_driver
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  lib
  ${catkin_INCLUDE_DIRS}
)


link_directories(lib)






#add_executable(ars_radar_node src/ars_radar_node.cpp)
#target_link_libraries(ars_radar_node ${catkin_LIBRARIES})

add_executable(ars_radar src/ars_radar.cpp)
target_link_libraries(ars_radar ${catkin_LIBRARIES})
