#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Description: Basic Grid Mapper based on Line Status and Odometry.

import rospy
import numpy as np
import math
from tf.transformations import euler_from_quaternion

# ROS Msgs
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Int8, Header

class SlamMapperNode: 
    def __init__(self):
        node_name = 'slam_mapper_node' 
        rospy.init_node(node_name, anonymous=True) 
        rospy.loginfo(f"[{node_name}] Initializing...")

        # --- Parameters ---
        self.map_resolution = rospy.get_param("~map_resolution", 0.05)
        self.map_width_meters = rospy.get_param("~map_width_meters", 10.0)
        self.map_height_meters = rospy.get_param("~map_height_meters", 10.0)
        self.map_publish_rate = rospy.get_param("~map_publish_rate", 1.0)
        self.map_frame_id = rospy.get_param("~map_frame_id", "odom")
        self.mark_distance_y = rospy.get_param("~mark_distance_y", 0.15)
        self.mark_distance_x = rospy.get_param("~mark_distance_x", 0.1)
        self.mark_radius_cells = rospy.get_param("~mark_radius_cells", 0)
        self.value_free = 0
        self.value_yellow_line = 50
        self.value_white_line = 100

        # --- Map Initialization ---
        self.map_width_cells = int(round(self.map_width_meters / self.map_resolution))
        self.map_height_cells = int(round(self.map_height_meters / self.map_resolution))
        self.map_origin_x = -self.map_width_meters / 2.0
        self.map_origin_y = -self.map_height_meters / 2.0
        self.map_origin_z = 0.0
        self.map_data = np.full((self.map_height_cells, self.map_width_cells),
                                self.value_free, dtype=np.int8)

        rospy.loginfo(f"[{node_name}] Map initialized to FREE: {self.map_width_cells}x{self.map_height_cells} cells @ {self.map_resolution} m/cell")

        # --- State Variables ---
        self.current_pose = None
        self.left_line_status = -1
        self.right_line_status = -1

        # --- ROS Comms ---
        self.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
        self.odom_sub = rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback, queue_size=1)
        # Remap these topics in the launch file if the detector node has a different name/namespace
        self.left_status_sub = rospy.Subscriber("~line/left/status", Int8, self.left_status_callback, queue_size=1)
        self.right_status_sub = rospy.Subscriber("~line/right/status", Int8, self.right_status_callback, queue_size=1)
        self.map_publish_timer = rospy.Timer(rospy.Duration(1.0 / self.map_publish_rate), self.publish_map)

        rospy.loginfo(f"[{node_name}] Subscribed to /odometry/filtered, ~line/left/status, ~line/right/status")
        rospy.loginfo(f"[{node_name}] Publishing map to /map")


    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.update_map_based_on_status()

    def left_status_callback(self, msg):
        self.left_line_status = msg.data

    def right_status_callback(self, msg):
        self.right_line_status = msg.data

    def world_to_map_coords(self, world_x, world_y):
       
        if self.map_resolution == 0: return None, None
        map_x = int(round((world_x - self.map_origin_x) / self.map_resolution))
        map_y = int(round((world_y - self.map_origin_y) / self.map_resolution))
        if 0 <= map_x < self.map_width_cells and 0 <= map_y < self.map_height_cells:
            return map_x, map_y
        else: return None, None

    def update_map_based_on_status(self):
        
        if self.current_pose is None: return
        pos = self.current_pose.position; orient_q = self.current_pose.orientation
        orient_list = [orient_q.x, orient_q.y, orient_q.z, orient_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orient_list)
        robot_x, robot_y, robot_yaw = pos.x, pos.y, yaw
        points_to_mark = []
        if self.left_line_status != -1:
            x_rel = self.mark_distance_x; y_rel = self.mark_distance_y
            target_x = robot_x + x_rel * math.cos(robot_yaw) - y_rel * math.sin(robot_yaw)
            target_y = robot_y + x_rel * math.sin(robot_yaw) + y_rel * math.cos(robot_yaw)
            occ_value = self.value_white_line if self.left_line_status == 0 else self.value_yellow_line
            points_to_mark.append((target_x, target_y, occ_value))
        if self.right_line_status != -1:
            x_rel = self.mark_distance_x; y_rel = -self.mark_distance_y
            target_x = robot_x + x_rel * math.cos(robot_yaw) - y_rel * math.sin(robot_yaw)
            target_y = robot_y + x_rel * math.sin(robot_yaw) + y_rel * math.cos(robot_yaw)
            occ_value = self.value_white_line if self.right_line_status == 0 else self.value_yellow_line
            points_to_mark.append((target_x, target_y, occ_value))
        if points_to_mark:
            for wx, wy, val in points_to_mark:
                cell_x, cell_y = self.world_to_map_coords(wx, wy)
                if cell_x is not None:
                    for dx in range(-self.mark_radius_cells, self.mark_radius_cells + 1):
                        for dy in range(-self.mark_radius_cells, self.mark_radius_cells + 1):
                            adj_x, adj_y = cell_x + dx, cell_y + dy
                            if 0 <= adj_x < self.map_width_cells and 0 <= adj_y < self.map_height_cells:
                                self.map_data[adj_y, adj_x] = val

    def publish_map(self, event=None):
    
        map_msg = OccupancyGrid()
        map_msg.header.stamp = rospy.Time.now(); map_msg.header.frame_id = self.map_frame_id
        map_msg.info.map_load_time = map_msg.header.stamp; map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width_cells; map_msg.info.height = self.map_height_cells
        map_msg.info.origin.position.x = self.map_origin_x; map_msg.info.origin.position.y = self.map_origin_y
        map_msg.info.origin.position.z = self.map_origin_z; map_msg.info.origin.orientation.w = 1.0
        map_msg.data = self.map_data.ravel().tolist()
        self.map_pub.publish(map_msg)


if __name__ == '__main__':
    try:
        mapper = SlamMapperNode() 
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Grid mapper node shut down.")
    except Exception as main_e:
        rospy.logfatal(f"Unhandled exception in grid mapper main: {main_e}")