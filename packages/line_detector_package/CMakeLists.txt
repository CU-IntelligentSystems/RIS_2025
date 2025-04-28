cmake_minimum_required(VERSION 2.8.3)
project(line_detector_package)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  duckietown_msgs
  tf
  
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs duckietown_msgs tf line_detector
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(FILES config/line_detector_config.yaml
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
