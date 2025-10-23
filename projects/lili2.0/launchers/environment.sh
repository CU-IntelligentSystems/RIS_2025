#!/usr/bin/env bash
export VEHICLE_NAME=paul
export ROS_MASTER_URI=http://lili-2.local:11311
# Source ROS Melodic
source /opt/ros/melodic/setup.bash
# Source your catkin workspace inside the container
source /code/catkin_ws/devel/setup.bash

