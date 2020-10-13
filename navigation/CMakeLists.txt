cmake_minimum_required(VERSION 2.8.3)
project(navigation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_ros
  roscpp
  rospy
  tf
)

catkin_package()


include_directories(
  ${catkin_INCLUDE_DIRS}
)
