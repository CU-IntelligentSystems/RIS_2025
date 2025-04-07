#!/bin/bash

# Source the Duckietown environment
source /environment.sh

# Initialize the launch process (sets up environment variables, etc.)
dt-launchfile-init

# Run the wheel encoder subscriber node
rosrun my_package wheel_encoder_reader_node.py

# Wait for the node to finish (it should run indefinitely)
dt-launchfile-join
