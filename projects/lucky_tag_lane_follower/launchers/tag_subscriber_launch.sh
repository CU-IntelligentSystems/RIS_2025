#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun luckyduckie_tag_lf tag_subscriber.py

# wait for app to end
dt-launchfile-join