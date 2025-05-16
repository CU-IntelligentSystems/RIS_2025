# AprilTag Detection with Duckiebot 

This repository is based on the [Duckietown ROS Template](https://github.com/duckietown/template-ros) and is used to detect AprilTags on a Duckiebot. The robot can interpret tag positions and publish relevant messages or transformations. Originally intended for SLAM, this project shifted focus solely to AprilTag detection and reaction due to time constraints.

---

##  Quick Start

Ensure You are running the commands below in separate terminals.

### Build the container

```bash
dts devel build -H lucky -f
```

### Run the publisher node

```bash
dts devel run -H lucky -L tag_publisher_launch -n publisher
```

### Run the subscriber node

```bash
dts devel run -H lucky -L tag_subscriber_launch -n subscriber
```

---

##  Functionality

The system uses the robot's camera to:

- Detect AprilTags using the `apriltag` Python library.
- Estimate each tag's 3D pose using `cv2.solvePnP`.
- Publish:
  - Tag detections on `/apriltag_detections`
  - Estimated poses on `/apriltag_poses`
  - Semantic string messages on `/apriltag_messages`
- Broadcast TF transforms between frames.

### Example tag-based messages

| Tag ID | Message |
|--------|---------|
| 2      | "lucky says to yield" |
| 6      | "lucky says to go right" |
| 7      | "lucky says to go left" |
| 20     | "lucky says to stop" |
| 74     | "lucky says to wait for the traffic light" |
| 96     | "lucky says to slow down" |

---

##  Project Structure

```
├── assets/
├── docs/
├── html/ 
│  
│  
├── launchers/
│   ├── default.sh
│   ├── tag_publisher_launch.sh
│   └── tag_subscriber_launch.sh
├── packages/
│   └── [ROS packages here]:luckieduckie_tag_lf
│       ├──src/
│       │  ├──tag_publisher.py
│       │  └──tag_publisher.py
│       ├──CMakeLists.txt
│       └── package.xml
├── dependencies-apt.txt
├── dependencies-py3.txt
├── dependencies-py3.txt
└── README.md
```

---

##  Dependencies


###  APT Packages (in `dependencies-apt.txt`)

```txt
libopencv-dev
python3-opencv
ros-noetic-tf2-geometry-msgs
```

---

###  Python Packages (in `dependencies-py3.txt`)

```txt
apriltag
numpy
opencv-python
transforms3d
scipy>=1.7.0
```

##  Configuration Notes

- The camera calibration matrix and distortion coefficients need to be replaced with your own robot's calibration values. 
- add the name of your robot to the topics
- We used the default tag size: `0.065 m` (6.5 cm). 
- Make the python scripts executable `chmod +x`
---

