#!/bin/bash

# Source the Duckietown environment variables and ROS setup within the container
source /environment.sh

# Initialize the Duckietown launch process (handles signals, logging, etc.)
dt-launchfile-init

# This roslaunch command will start ekf_localization_node AND load our YAML parameters
sleep 5
LAUNCH_CMD="roslaunch Localization duckiebot_localization.launch"

echo "Executing: ${LAUNCH_CMD}"

# Execute the roslaunch command in the background
${LAUNCH_CMD} &

PID=$!
echo "Launched roslaunch with PID ${PID}"

dt-launchfile-join
