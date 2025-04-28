#!/bin/bash

# Source the Duckietown environment variables and ROS setup
source /environment.sh

# Initialize the Duckietown launch process (handles signals, logging, etc.)
dt-launchfile-init

# Define the command to run your white detection launch file
# This uses roslaunch to start the node(s) defined in your .launch file
LAUNCH_CMD="roslaunch White_detection white_detection.launch"
#                     ^^^^^^^^^^^^^^^^^ -- Your package name
#                                       ^^^^^^^^^^^^^^^^^^^^^^^^^^^^ -- Your XML launch file

# Print the command we are about to run (optional, but good for debugging)
echo "Executing: ${LAUNCH_CMD}"

# Execute the roslaunch command in the background
# Running in the background allows dt-launchfile-join to manage shutdown
${LAUNCH_CMD} &

# Capture the process ID of the roslaunch command (optional)
PID=$!
echo "Launched roslaunch with PID ${PID}"

# Use dt-launchfile-join to keep the script running and manage shutdown
# This will wait for signals (like SIGINT/SIGTERM from docker stop)
dt-launchfile-join

# Optional: Add cleanup commands here if needed after dt-launchfile-join exits
# echo "Cleaning up..."
# kill ${PID}