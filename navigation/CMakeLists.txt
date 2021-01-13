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

add_executable(navigate_to_wp_node src/navigate_to_wp.cpp)
target_link_libraries(navigate_to_wp_node ${catkin_LIBRARIES})