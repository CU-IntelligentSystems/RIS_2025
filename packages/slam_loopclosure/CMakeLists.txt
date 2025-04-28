cmake_minimum_required(VERSION 3.0.2)
project(slam_loopclosure)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
  cv_bridge
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  LaneLine.msg
  LaneLineArray.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs 
)

###################################
## Catkin package configuration  ##
###################################

# The catkin_package macro generates cmake config files for your package
# Declare things needed for dependent packages
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime geometry_msgs nav_msgs rospy sensor_msgs std_msgs tf cv_bridge
#  DEPENDS system_lib
)

###################################
## Install                       ##
###################################

# Python scripts
catkin_install_python(PROGRAMS
  scripts/line_status_detector_updated.py 
  scripts/slam_mapper_node_updated.py  
  scripts/pose_graph_optimizer_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Message files
install(DIRECTORY msg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
)



