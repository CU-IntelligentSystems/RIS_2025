#!/bin/bash

# Source the Duckietown environment
source /environment.sh

# Initialize the launch process
dt-launchfile-init

# Run the ROS camera reader node
rosrun my_package camera_reader_node.py

# Wait for the node to terminate
dt-launchfile-join
