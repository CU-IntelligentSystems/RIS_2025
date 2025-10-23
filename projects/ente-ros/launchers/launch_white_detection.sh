#!/bin/bash

# Source the Duckietown environment variables and ROS setup
source /environment.sh

# Initialize the Duckietown launch process
dt-launchfile-init

# This uses roslaunch to start the node(s) defined in our .launch file
LAUNCH_CMD="roslaunch White_detection white_detection.launch"
#                     ^^^^^^^^^^^^^^^^ -- our package name
#                                       ^^^^^^^^^^^^^^^^^^^^^^^^^^^^ -- our XML launch file


echo "Executing: ${LAUNCH_CMD}"

# Execute the roslaunch command in the background
# Running in the background allows dt-launchfile-join to manage shutdown
${LAUNCH_CMD} &

# Capture the process ID of the roslaunch command
PID=$!
echo "Launched roslaunch with PID ${PID}"

# keeps the script running
dt-launchfile-join