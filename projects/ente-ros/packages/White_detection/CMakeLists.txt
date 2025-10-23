cmake_minimum_required(VERSION 3.0.2)
project(White_detection)

# Find catkin packages required for runtime dependencies

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_geometry
  message_filters
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
  tf_conversions
)

catkin_package(

  CATKIN_DEPENDS # List runtime dependencies for other packages using this one's C++ libs/headers
)

# Install Python scripts
# This makes 'rosrun White_detection detection.py' work
catkin_install_python(PROGRAMS
  scripts/line_status_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
# This makes launch files in the 'launch/' directory available to 'roslaunch'
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


