#!/bin/bash

# This script launches the complete SLAM pipeline for the Duckiebot.
# It runs the main launch file which includes localization, line detection,
# the static TF override, and the mapping node.

# Source the Duckietown environment variables and ROS setup within the container
echo "Sourcing environment..."
source /environment.sh

# Initialize the Duckietown launch process
dt-launchfile-init

# --- Define the main launch command ---
# Ensure 'slam' is your package name and 'slam_pipeline.launch' is your main launch file.
# We pass 'veh:=ente' as the pipeline launch file might use it.
LAUNCH_CMD="roslaunch slam slam_pipeline.launch veh:=ente"

# Print the command we are about to run
echo "Executing: ${LAUNCH_CMD}"

# Execute the roslaunch command in the background
${LAUNCH_CMD} &

# Capture the process ID (optional)
PID=$!
echo "Launched roslaunch with PID ${PID}"

# Use dt-launchfile-join to keep the container running and manage shutdown
echo "Joining launch file... (Use Ctrl+C to stop)"
dt-launchfile-join

# Optional cleanup after shutdown
echo "Launch process finished."
# kill ${PID} # Uncomment if explicit kill is needed and PID capture works