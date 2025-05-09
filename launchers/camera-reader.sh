#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch camera_reader_node in background
rosrun my_package camera_reader_node.py 

# wait for all background processes to end
dt-launchfile-join
