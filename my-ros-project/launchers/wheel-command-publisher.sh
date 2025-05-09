#!/bin/bash

# sourcing environment variables
source /environment.sh

# initialize launch file / Initailizing Duckietown container lifecycle
dt-launchfile-init

# launch subscriber / starting the node
rosrun my_package wheel_command_publisher_node.py

# wait for app/process to end
dt-launchfile-join
