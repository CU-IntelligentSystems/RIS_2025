# EKF-SLAM in Duckietown World

This project implements and explores Simultaneous Localization and Mapping (SLAM) techniques on a Duckiebot platform. It combines Extended Kalman Filter (EKF) based localization with visual lane detection to create a map of the Duckiebot's environment.

## Prerequisites

* You must have the Duckietown development environment (Duckietown Shell, `dts`) installed and configured on your system. Please refer to the official [Duckietown documentation](https://docs.duckietown.org/) if you haven't set this up yet.
* A configured Duckiebot (either real or simulated) with a known hostname.

## Installation & Setup

1.  **Clone the Repository:**
    Get the project code onto your local machine.
    ```bash
    git clone https://github.com/Thendyom/ente-ros
    cd
    ```
   into the directory where you cloned it

2.  **Configure Your Duckiebot Name:**
    Before building, you need to tell the project the hostname of *your* specific Duckiebot. You must replace the default name (`ente`) with your bot's name in the following bash file:

    * **File:** `it_begins.sh`
        * **Find:** `LAUNCH_CMD="roslaunch slam slam_pipeline.launch veh:=ente"`
        * **Change `ente`** to `YOUR_DUCKIEBOT_NAME`

    *(Remember to replace `YOUR_DUCKIEBOT_NAME` with the actual hostname!)*

## Building the Docker Image

Now, build the Docker image required to run the code using the Duckietown Shell. Make sure to specify your Duckiebot's hostname using the `-H` flag. The `-f` flag forces a rebuild if needed.

```bash
# Replace YOUR_DUCKIEBOT_NAME with your bot's actual hostname
dts devel build -H YOUR_DUCKIEBOT_NAME -f

Then run the system
dts devel run -H YOUR_DUCKIEBOT_NAME -L it_begins
```
## Keyboard Control 
To control the bot, we suggest using the keyboard control. To do this, type: 
```bash
dts duckiebot keyboard_control YOUR_DUCKIEBOT_NAME
```
## Viewing
1) Open a new terminal and type
```bash
dts start_gui_tools
```
3) Then type 
```bash
rosrun rviz rviz
```
4) Switch to odom frame
5) Add TF data and open \map topic
6) Et voila! You can start to play around with your bot 

If you want to view the line detector output 
- Run rqt_image_view and observe the corresponding topics (line_status_detector/image/line_overlay)

## Saving the map 
The generated map files are saved directly on the Duckiebot. Follow these steps to access them:

Verify Connection: Ensure you can reach your Duckiebot on the network. You can test this using ping:

```bash

ping DuckieName.local
```
SSH into Duckiebot: Open a terminal and connect to the Duckiebot using SSH:
```bash

ssh duckie@DuckieName.local
```
(Again, replace YOUR_DUCKIEBOT_NAME with the correct hostname)

Enter default Password: 

```bash
quackquack
```

Navigate to Maps Directory: Once logged in, change to the directory to see where the maps are stored:
```bash

cd /data/DuckieName/maps
ls
```
## Transfer the map to your local file 

Exit the ssh connection:
```bash
exit
```

Create a directory on your machine and transfer them form the Duckie to that directory:
```bash
mkdir -p ~/duckie_maps
scp duckie@DuckieName.local:/data/DuckieName/maps/* ~/duckie_maps/
```
## Map File Name Formatting
The generated map files consist of two parts: a .pgm image file and a .yaml metadata file. 
They follow the naming convention DuckieName_map_DDMMYYYY_HHMM.

Example:
For a map saved on May 2nd, 2025 at 2:59 PM on a Duckiebot named ente, the files would be:

    ente_map_02May2025_1459.pgm
    ente_map_02May2025_1459.yaml





