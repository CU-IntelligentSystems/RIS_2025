cmake_minimum_required(VERSION 3.0.2)
project(Localization)

# Find catkin and required build dependencies/components
# Added duckietown_msgs and tf as they are used by the Python script
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  duckietown_msgs
  tf
  # robot_localization # Typically not needed as a component unless compiling against its headers
)

# Declare package build information and runtime dependencies
catkin_package(
  CATKIN_DEPENDS
    robot_localization
    nav_msgs
    sensor_msgs
    geometry_msgs
    tf2_ros
    rospy
    std_msgs
    duckietown_msgs
    tf
)

# Specify include directories for C++ compilation
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# === Installation Rules ===

# Install Python script(s)
install(PROGRAMS
  scripts/duckie_odom_converter.py
  scripts/imu_covariance_fixer.py # <<< Add this line
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)