#!/bin/bash

source /environment.sh

# Initialize Duckietown launch process
dt-launchfile-init

# Launch camera node
rosrun my_package camera_reader_node.py &

# Launch PID node
rosrun my_package my_subscriber_node.py &

# Launch stop/acceleration node
rosrun my_package my_publisher_node.py &

# Launch the selector node
rosrun my_package twist_mux_selector.py &

# Wait for all background processes
dt-launchfile-join
