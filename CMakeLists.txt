cmake_minimum_required(VERSION 3.0.2)
project(groundSeg)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES groundSeg
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  include/
  src/
  ${catkin_INCLUDE_DIRS}
)

add_executable(main src/main.cpp src/groundSeg.cpp include/groundSeg.h)
target_link_libraries(main ${catkin_LIBRARIES})
