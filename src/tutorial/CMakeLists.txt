cmake_minimum_required(VERSION 3.0.2)
project(tutorial)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  fs_msgs
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  message_runtime
)

## Declare ROS messages, services and actions
#add_message_files(
#  FILES
##  Track.msg
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  fs_msgs
)

## catkin specific configuration
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tutorial
  CATKIN_DEPENDS fs_msgs roscpp rospy std_msgs geometry_msgs message_runtime
)

## Build
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(data_logger src/data_logger.cpp)
add_dependencies(data_logger ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(data_logger ${catkin_LIBRARIES})

## Install
# catkin_install_python(...)
# install(TARGETS ...)
