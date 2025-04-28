#!/bin/bash


# Source the Duckietown environment variables and ROS setup within the container
echo "Sourcing environment..."
source /environment.sh || { echo "Failed to source environment.sh"; exit 1; }

# Initialize the Duckietown launch process
# This sets up log directories and other necessary components.
echo "Initializing dt-launchfile..."
dt-launchfile-init || { echo "dt-launchfile-init failed"; exit 1; }

LAUNCH_CMD="roslaunch slam_loopclosure slam_loopclosure.launch veh:=${VEHICLE_NAME:-ente}" # Use VEHICLE_NAME env var or default to 'ente'

# Print the command we are about to run
echo "Executing: ${LAUNCH_CMD}"

# Execute the roslaunch command in the background
${LAUNCH_CMD} &

# Capture the process ID (PID) of the roslaunch command
PID=$!
echo "Launched roslaunch with PID ${PID}"

# Use dt-launchfile-join to keep the container running.
# It waits for the launched process (or ROS master) to terminate
# and handles cleanup signals (like Ctrl+C) gracefully.
echo "Joining launch file (waiting for termination)..."
dt-launchfile-join || { echo "dt-launchfile-join failed"; exit 1; }

# This message is shown after the launch process finishes (e.g., after Ctrl+C)
echo "Launch process finished."

# Exit cleanly
exit 0

