# EKF-SLAM in Duckietown World

This project implements and explores Simultaneous Localization and Mapping (SLAM) techniques on a Duckiebot platform. It combines Extended Kalman Filter (EKF) based localization with visual lane detection to create a map of the Duckiebot's environment.

## Prerequisites

* You must have the Duckietown development environment (Duckietown Shell, `dts`) installed and configured on your system. Please refer to the official [Duckietown documentation](https://docs.duckietown.org/) if you haven't set this up yet.
* A configured Duckiebot (either real or simulated) with a known hostname.

## Installation & Setup

1.  **Clone the Repository:**
    Get the project code onto your local machine.
    ```bash
    # Replace <your-repository-url> with the actual URL
    git clone <your-repository-url>
    cd <repository-directory-name>
    ```

2.  **Configure Your Duckiebot Name:**
    Before building, you need to tell the project the hostname of *your* specific Duckiebot. You must replace the default name (`ente`) with your bot's name in the following files:

    * **File:** `packages/White_detection/launch/white_detection.launch`
        * **Find:** `<arg name="veh" default="ente" ... />`
        * **Change `ente`** to `YOUR_DUCKIEBOT_NAME`

    * **File:** `packages/localization/launch/duckiebot_localization.launch`
        * **Find:** `<arg name="veh" default="ente" ... />`
        * **Change `ente`** to `YOUR_DUCKIEBOT_NAME`

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

## Viewing
1) Open a new terminal and type dts start_gui_tools
2) Then type rosrun rviz rviz 
3) Switch to odom frame
4) Add TF data and open \map topic
5) Et voila! You can start to play around with your bot 

If you want to view the line detector output 
- Run rqt_image_view and observe the corresponding topics (line_status_detector/image/line_overlay)
