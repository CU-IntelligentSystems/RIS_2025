cmake_minimum_required(VERSION 3.0.2)
project(luckyduckie_tag_lf)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  duckietown_msgs
  tf2_ros
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
