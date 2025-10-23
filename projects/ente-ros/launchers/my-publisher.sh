#!/bin/bash

source /environment.sh

# Initialize the launch process
dt-launchfile-init

# Run the ROS publisher node
rosrun my_package my_publisher_node.py

# Wait for the application to end
dt-launchfile-join
