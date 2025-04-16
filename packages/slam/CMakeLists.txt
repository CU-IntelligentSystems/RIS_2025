cmake_minimum_required(VERSION 3.0.2)
project(slam) # CHANGED

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    rospy
    std_msgs
    tf
)

# Install Python script (make sure filename matches)
catkin_install_python(PROGRAMS
  scripts/slam_mapper_node.py # CHANGED FILENAME (OPTIONAL)
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)