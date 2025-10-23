cmake_minimum_required(VERSION 2.8.3)
project(line_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  duckietown_msgs
  cv_bridge
)

#-----------------------------------------------------------------
#  Exported API
#-----------------------------------------------------------------
catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES line_detector
  # CATKIN_DEPENDS roscpp rospy duckietown_msgs cv_bridge
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#-----------------------------------------------------------------
#  Install Python script(s)
#-----------------------------------------------------------------
# Now that catkin_package() has run, CATKIN_PACKAGE_BIN_DESTINATION is defined.
catkin_install_python(
  PROGRAMS scripts/line_detection_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

