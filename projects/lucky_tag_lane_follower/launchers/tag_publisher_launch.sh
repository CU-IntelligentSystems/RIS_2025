#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch publisher
rosrun luckyduckie_tag_lf tag_publisher.py

# wait for app to end
dt-launchfile-join
