#!/bin/bash

source /environment.sh

# Initialize the launch process
dt-launchfile-init

roslaunch line_detector line_detector_node.launch veh:=ente

# Wait for the application to end
dt-launchfile-join

