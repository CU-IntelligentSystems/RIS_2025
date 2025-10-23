#!/bin/bash

# Source the Duckietown environment
source /environment.sh

# Initialize the launch process
dt-launchfile-init

# Run your ROS subscriber node
rosrun my_package my_subscriber_node.py

# Wait for the application to end
dt-launchfile-join
