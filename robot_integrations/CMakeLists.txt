cmake_minimum_required(VERSION 3.0.2)
project(robot_integrations)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
)

## Declare ROS messages, services and actions ##
add_service_files(
  FILES
  Point.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom_imu2 src/odom_imu2.cpp)
target_link_libraries(odom_imu2 ${catkin_LIBRARIES})