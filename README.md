# DuckieBot Autonomous Lane Following

**Author:** Mohamed Goda Ebrahim  
**Course:** S25_CO-548-A_RIS Project  
**Instructor:** Kristina Nikolovska  
**Submission Date:** 22nd of August  
**Group Members:** None  
**GitHub Repository:** [https://github.com/muhameddgoda/duckiebot-lane-following](https://github.com/muhameddgoda/duckiebot-lane-following)

---

## Table of Contents

1. [Project Overview](#project-overview)  
2. [Features](#features)  
3. [Installation](#installation)  
4. [Usage](#usage)  
5. [Implementation Details](#implementation-details)  
6. [Simulation Environment](#simulation-environment)  
7. [Results](#results)  
8. [Future Work](#future-work)  
9. [References](#references)

---

## Project Overview

This project implements an autonomous lane following system for the **DuckieBot platform**. The system enables the robot to navigate yellow lanes in simulation using **ROS 2**, **OpenCV**, and **Gazebo**. The implementation focuses on:

- Real-time lane detection using computer vision  
- Adaptive speed control based on path curvature  
- Proportional steering control for smooth navigation  
- Testing in a comprehensive Gazebo simulation environment

The project demonstrates the core principles of autonomous navigation and provides a foundation for real-world robotic applications.

---

## Features

- HSV-based yellow lane detection for robust vision  
- Adaptive velocity control:
  - Straight lanes: 0.5 m/s  
  - Moderate curves: 0.15 m/s  
  - Sharp turns: 0.08 m/s  
- Proportional steering controller for lane centering  
- Real-time debugging visualization in simulation  
- Comprehensive test track with S-curves, chicanes, sharp turns, and long straights

---

## Installation

1. **Clone the repository:**
   git clone https://github.com/muhameddgoda/duckiebot-lane-following.git
   cd duckiebot-lane-following


2. **Setup ROS 2 environment:**

   * Make sure ROS 2 Humble is installed
   * Source the ROS 2 workspace:

   source /opt/ros/humble/setup.bash
   ```

3. **Install Python dependencies:**

   pip install -r requirements.txt
   ```

4. **Run Gazebo simulation environment:**

   ros2 launch duckiebot_lane_following lane_following_launch.py
   ```

---

## Usage

1. Launch the DuckieBot simulation environment in Gazebo
2. Start the lane following node:

   ```bash
   ros2 run duckiebot_lane_following simple_lane_follower.py
   ```
3. Monitor the robot navigating the track in real-time
4. Debugging visualization is displayed via the ROS 2 image topic

---

## Implementation Details

* **Lane Detection:**

  * HSV color space filtering for yellow lane detection
  * Morphological operations for noise reduction
  * Contour analysis to calculate lane center

* **Adaptive Velocity Control:**

  * Velocity adjusted based on path curvature and steering error
  * Ensures safe navigation in sharp turns and efficiency on straights

* **Steering Control:**

  * Proportional controller based on lane center deviation
  * Steering gain tuned for smooth cornering

---

## Simulation Environment

* Custom Gazebo test track: `connected_zigzag_track.world`

* Features:

  * Long straights for speed testing
  * Gentle and sharp curves for steering evaluation
  * S-curves and chicanes for complex maneuvering

* Designed to validate both lane detection and adaptive velocity control performance

---

## Results

* Reliable lane following in Gazebo simulation
* Smooth navigation through curves, S-turns, and chicanes
* Adaptive speed successfully prevents overshooting during sharp turns
* Visualization allows monitoring lane detection and control in real-time

---

## Future Work

* Enhance lane detection with machine learning for robustness under varying lighting and obstacles
* Upgrade steering controller to PID for smoother handling
* Deploy the system on physical DuckieBot hardware
* Add dynamic obstacle detection and avoidance

---

## References

1. Duckietown Documentation: [https://docs.duckietown.org](https://docs.duckietown.org)
2. ROS 2 Documentation: [https://docs.ros.org/en/humble/](https://docs.ros.org/en/humble/)
3. OpenCV Documentation: [https://docs.opencv.org/](https://docs.opencv.org/)
4. Gazebo Tutorials: [http://gazebosim.org/tutorials](http://gazebosim.org/tutorials)
5. Lefèvre, T., et al. *Autonomous Vehicles: Detection and Control*, 2020
