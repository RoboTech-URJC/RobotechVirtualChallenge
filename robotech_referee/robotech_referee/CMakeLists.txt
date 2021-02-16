cmake_minimum_required(VERSION 3.0.2)
project(robotech_referee)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
  robotech_referee_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robotech_referee
#  CATKIN_DEPENDS cv_bridge roscpp rospy sensor_msgs/image std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/robotech_referee.cpp
# )

add_executable(challenge_starter_node src/challenge_starter.cpp)
target_link_libraries(challenge_starter_node ${catkin_LIBRARIES})

add_executable(task_starter_node src/task_starter.cpp)
target_link_libraries(task_starter_node ${catkin_LIBRARIES})

add_executable(speed_checker_node src/speed_checker.cpp)
target_link_libraries(speed_checker_node ${catkin_LIBRARIES})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robotech_referee.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
