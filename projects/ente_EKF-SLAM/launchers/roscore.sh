#!/bin/bash
# Source the Duckietown environment
source /environment.sh

# Initialize the launch process
dt-launchfile-init

# Start the ROS master
roscore

# Wait for the process to end (roscore runs indefinitely)
dt-launchfile-join
