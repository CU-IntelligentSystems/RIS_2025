#!/usr/bin/env bash
# =====================================
# Launcher: run_white_line_follower.sh
# Sets up the ROS environment and starts
# the white-line detection & follower node.

# 1. Source your catkin workspace
cd /code
source environment.sh

# 2. (Optional) If launching remotely from your laptop:
# export ROS_MASTER_URI=http://<duckiebot-hostname>.local:11311
# export ROS_IP=$(hostname -I | awk '{print $1}')

# 3. Run the ROS launch file
roslaunch line_detection line_detection_node.launch
roslaunch camera_reader camera_reader.launch &
roslaunch wheel_control wheel_control.launch &
roslaunch twist_control twist_control.launch &


