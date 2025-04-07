#!/bin/bash

# Source the Duckietown environment
source /environment.sh

# Initialize the launch process
dt-launchfile-init

# Run the IMU reader node
rosrun my_package imu_reader_node.py

# Wait for the node to finish
dt-launchfile-join
