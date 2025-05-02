#!/bin/bash

# This script launches the complete SLAM pipeline for the Duckiebot.
# It runs the main launch file which includes localization, line detection,
# the static TF override, and the mapping node.

# Source the Duckietown environment variables and ROS setup within the container
echo "Sourcing environment..."
source /environment.sh

# Initialize the Duckietown launch process
dt-launchfile-init

# this is what we want to launch 
# veh is a variable for the name of the bot
LAUNCH_CMD="roslaunch slam slam_pipeline.launch veh:=ente"

# Print the command we are about to run
echo "Executing: ${LAUNCH_CMD}"

# Executes the roslaunch command in the background
${LAUNCH_CMD} &

# Capture the process ID
PID=$!
echo "Launched roslaunch with PID ${PID}"

# Use dt-launchfile-join to keep the container running and manage shutdown
echo "Joining launch file..."
dt-launchfile-join

# cleanup after shutdown
echo "Launch process finished."
