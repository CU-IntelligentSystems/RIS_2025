#!/bin/bash

# Source the Duckietown environment variables and ROS setup within the container
source /environment.sh

# Initialize the Duckietown launch process (handles signals, logging, etc.)
dt-launchfile-init

# Define the command to run YOUR localization launch file
# This roslaunch command will start ekf_localization_node AND load your YAML parameters
sleep 5
LAUNCH_CMD="roslaunch Localization duckiebot_localization.launch"
#                         ^^^^^^^^^^^^ -- Your package name
#                                      ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ -- Your XML launch file

# Print the command we are about to run (optional, but good for debugging)
echo "Executing: ${LAUNCH_CMD}"

# Execute the roslaunch command in the background
${LAUNCH_CMD} &

# Capture the process ID of the roslaunch command
# (Optional, but useful if dt-launchfile-join needs it or for specific cleanup)
PID=$!
echo "Launched roslaunch with PID ${PID}"

# Wait for the ROS nodes to be initialized (optional delay)
# sleep 5

# Use dt-launchfile-join to keep the script running and manage shutdown
# This will wait for signals (like SIGINT/SIGTERM) and then potentially clean up
# It might also monitor the background process depending on its implementation
dt-launchfile-join

# Optional: Add cleanup commands here if needed after dt-launchfile-join exits
# echo "Cleaning up..."
# kill ${PID}