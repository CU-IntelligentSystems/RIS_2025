#!/bin/bash

# Source environment variables
source /environment.sh

# initialize launch file
dt-launchfile-init

# Start nodes
rosrun my_package camera_reader_node.py > camera.log 2>&1 &
rosrun my_package led_controller.py > led.log 2>&1 &
rosrun my_package red_line_detector.py > red_line.log 2>&1 &
rosrun my_package wheel_controller.py > wheel.log 2>&1 &
# Wait for processes to finish

# wait for app to end
dt-launchfile-join

wait
