Duckietown Gazebo — Quick Start 
================================================
https://drive.google.com/file/d/1Ac-K2349BRdY-3yQVffOAFaB_4hKZ-es/view?usp=sharing

download duckietowne_gazebo.tar.gz from here

load the image using 

docker load -i duckietowne_gazebo.tar.gx
----------------------------
Verify image with the command 

docker images

then use

docker run -it --rm \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    duckietowne_gazebo.tar.gz:<replace_with_your_tag>
    


-------------------------
if image is already running:

docker ps
docker exec -it <container_id_or_name> bash     # e.g., f905b6a61b75

Baseline env (run in every shell you use)
-----------------------------------------
source /opt/ros/kinetic/setup.bash
source /root/duckietown/catkin_ws/devel/setup.bash
cd /root/duckietown/catkin_ws/src/duckietown-sim-server/duckietown_gazebo
source env_gazebo.sh
export GAZEBO_MODEL_DATABASE_URI=""   # force local models

Key paths in the repo
---------------------
Launch files:  /root/duckietown/catkin_ws/src/duckietown-sim-server/duckietown_gazebo/launch/
Worlds:        /root/duckietown/catkin_ws/src/duckietown-sim-server/duckietown_gazebo/worlds/
Models:        /root/duckietown/catkin_ws/src/duckietown-sim-server/duckietown_gazebo/models/
Env script:    /root/duckietown/catkin_ws/src/duckietown-sim-server/duckietown_gazebo/env_gazebo.sh

Common worlds available
-----------------------
duckietown_loop.world
duckietown_singleStreet.world
duckietown_udem.world
mybotorg.world

Launch Gazebo + robot (GUI)
---------------------------
# Default launch (after we added world_name support)
roslaunch duckietown_gazebo duckietown.launch \
  world_name:=$(rospack find duckietown_gazebo)/worlds/duckietown_udem.world \
  x:=1.5 y:=1.0 z:=0.06 yaw:=0.0 paused:=false gui:=true headless:=false

# Headless
roslaunch duckietown_gazebo duckietown.launch \
  world_name:=$(rospack find duckietown_gazebo)/worlds/duckietown_udem.world \
  paused:=false gui:=false headless:=true

Map troubleshooting
-------------------
# Make sure Gazebo sees local assets
echo "$GAZEBO_MODEL_PATH"       # should include .../duckietown_gazebo/models
echo "$GAZEBO_RESOURCE_PATH"    # should include .../duckietown_gazebo/worlds

# (Optional) vendor models so Gazebo always finds them
mkdir -p ~/.gazebo/models
cp -r /root/duckietown/catkin_ws/src/duckietown-sim-server/duckietown_gazebo/models/* ~/.gazebo/models/

Sanity checks after launch
--------------------------
rostopic hz /clock              # ~100 Hz if unpaused
rostopic list | grep gazebo
rostopic info /cmd_vel          # Subscriber should be: /gazebo
rostopic hz /odom               # ~100 Hz

Read current robot velocity
---------------------------
# From odom (robot-frame)
rostopic echo -n1 /odom/twist/twist
rostopic echo -n1 /odom/twist/twist/linear/x      # forward m/s
rostopic echo -n1 /odom/twist/twist/angular/z     # yaw rate rad/s

Command velocity (Twist) — reliable YAML forms
----------------------------------------------
# Move forward 0.20 m/s (stop with Ctrl-C or a zero cmd)
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist -- \
'{linear: {x: 0.20, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Rotate in place 0.5 rad/s
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist -- \
'{linear: {x: 0.0,  y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'

# STOP (one-shot zero; all fields default to 0.0)
rostopic pub --once /cmd_vel geometry_msgs/Twist '{}'

# Bounded motion (auto-stops after 2s)
timeout 2s rostopic pub -r 10 /cmd_vel geometry_msgs/Twist -- \
'{linear: {x: 0.20, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

Pause / Unpause / Reset
-----------------------
rosservice call /gazebo/pause_physics "{}"
rosservice call /gazebo/unpause_physics "{}"

========================================================================================================
Gym-duckietown

install as usual 

git clone https://github.com/duckietown/gym-duckietown.git
cd gym-duckietown
pip3 install -e .


Then edit the setup.py so that the numpy version is not asserted
then install all the dependencied the requirements.txt in this repo
