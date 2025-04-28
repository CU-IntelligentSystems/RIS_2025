#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Description: Builds an Occupancy Grid Map based on detected LaneLine features
#              transformed using filtered odometry. Also detects potential
#              loop closures based on pose proximity.

import rospy
import numpy as np
import math
import tf.transformations as tf_trans
from skimage.draw import line as draw_line # For drawing lines on grid

# ROS Msgs
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
# Custom Msgs (Assuming they are in 'your_package_name')
# Replace 'your_package_name' with the actual name of your package
try:
    from slam_loopclosure.msg import LaneLine, LaneLineArray
except ImportError:
    rospy.logfatal("Could not import LaneLine messages. "
                   "Make sure the package is built and sourced, "
                   "and replace 'your_package_name' with the correct package name.")
    exit()

class FeatureMapperNode: # Renamed class
    def __init__(self):
        node_name = 'feature_mapper_node' # Renamed node
        rospy.init_node(node_name, anonymous=True)
        rospy.loginfo(f"[{node_name}] Initializing...")

        # --- Parameters ---
        self.map_resolution = rospy.get_param("~map_resolution", 0.05) # meters/cell
        self.map_width_meters = rospy.get_param("~map_width_meters", 20.0) # Larger map
        self.map_height_meters = rospy.get_param("~map_height_meters", 20.0)
        self.map_publish_rate = rospy.get_param("~map_publish_rate", 0.5) # Slower publish rate
        self.map_frame_id = rospy.get_param("~map_frame_id", "map") # Use 'map' frame
        self.odom_frame_id = rospy.get_param("~odom_frame_id", "odom") # Odometry frame
        self.line_segment_length = rospy.get_param("~line_segment_length", 0.3) # meters to draw lines
        self.loop_closure_dist_thresh = rospy.get_param("~loop_closure_dist_thresh", 0.5) # meters
        self.loop_closure_min_keyframes = rospy.get_param("~loop_closure_min_keyframes", 15) # Don't check recent frames

        # Map grid values
        self.value_unknown = -1 # Or 50? Standard is -1
        self.value_free = 0
        self.value_occupied_white = 100
        self.value_occupied_yellow = 90 # Slightly different value

        # --- Map Initialization ---
        self.map_width_cells = int(round(self.map_width_meters / self.map_resolution))
        self.map_height_cells = int(round(self.map_height_meters / self.map_resolution))
        # Map origin is bottom-left corner in world coordinates
        self.map_origin_x = -self.map_width_meters / 2.0
        self.map_origin_y = -self.map_height_meters / 2.0
        self.map_origin_z = 0.0
        # Initialize map to UNKNOWN
        self.map_data = np.full((self.map_height_cells, self.map_width_cells),
                                self.value_unknown, dtype=np.int8)

        rospy.loginfo(f"[{node_name}] Map initialized: {self.map_width_cells}x{self.map_height_cells} cells @ {self.map_resolution} m/cell")

        # --- State Variables ---
        self.current_odom_pose = None # Latest Pose from /odometry/filtered
        self.keyframes = [] # List to store (id, pose) tuples for loop closure check
        self.keyframe_counter = 0
        self.last_keyframe_pose = None
        self.keyframe_dist_thresh = rospy.get_param("~keyframe_dist_thresh", 0.3) # meters
        self.keyframe_angle_thresh = rospy.get_param("~keyframe_angle_thresh", np.deg2rad(10)) # radians


        # --- ROS Comms ---
        self.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
        # Subscribe to filtered odometry (likely from robot_localization)
        self.odom_sub = rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback, queue_size=5)
        # Subscribe to the new LaneLineArray topic from the detector node
        self.lines_sub = rospy.Subscriber("/line_parameter_detector/lines", LaneLineArray, self.lines_callback, queue_size=1)

        self.map_publish_timer = rospy.Timer(rospy.Duration(1.0 / self.map_publish_rate), self.publish_map)

        rospy.loginfo(f"[{node_name}] Subscribed to /odometry/filtered and /line_parameter_detector/lines")
        rospy.loginfo(f"[{node_name}] Publishing map to /map (frame: {self.map_frame_id})")


    def odom_callback(self, msg: Odometry):
        """ Stores the latest filtered odometry pose and checks for keyframes. """
        # Ensure the message is in the expected odom frame
        # Note: robot_localization usually publishes odom->base_link,
        # but the pose in the message itself is relative to the odom_frame_id specified in its config.
        # We assume the pose is relative to self.odom_frame_id here.
        if msg.header.frame_id != self.odom_frame_id:
             rospy.logwarn_throttle(10.0, f"[{rospy.get_name()}] Received odometry in frame '{msg.header.frame_id}' but expected '{self.odom_frame_id}'. Check robot_localization output frame.")
             # Attempt to use it anyway, assuming it represents odom->base_link state
             # self.current_odom_pose = msg.pose.pose # This might be incorrect if TF is not set up

        # For simplicity, assume the pose in the message IS the state in the odom frame
        self.current_odom_pose = msg.pose.pose

        # --- Keyframe and Loop Closure Check ---
        if self.current_odom_pose is not None:
            current_pos = self.current_odom_pose.position
            current_q = self.current_odom_pose.orientation
            _, _, current_yaw = tf_trans.euler_from_quaternion([current_q.x, current_q.y, current_q.z, current_q.w])

            add_keyframe = False
            if self.last_keyframe_pose is None:
                add_keyframe = True
            else:
                last_pos = self.last_keyframe_pose.position
                last_q = self.last_keyframe_pose.orientation
                _, _, last_yaw = tf_trans.euler_from_quaternion([last_q.x, last_q.y, last_q.z, last_q.w])

                dist = math.sqrt((current_pos.x - last_pos.x)**2 + (current_pos.y - last_pos.y)**2)
                angle_diff = abs(current_yaw - last_yaw)
                # Normalize angle difference
                while angle_diff > math.pi: angle_diff -= 2 * math.pi
                while angle_diff <= -math.pi: angle_diff += 2 * math.pi
                angle_diff = abs(angle_diff)

                if dist > self.keyframe_dist_thresh or angle_diff > self.keyframe_angle_thresh:
                    add_keyframe = True

            if add_keyframe:
                kf_id = self.keyframe_counter
                kf_pose = self.current_odom_pose # Store the pose associated with this keyframe
                self.keyframes.append((kf_id, kf_pose))
                self.last_keyframe_pose = kf_pose
                self.keyframe_counter += 1
                rospy.logdebug(f"[{rospy.get_name()}] Added keyframe {kf_id} at x={kf_pose.position.x:.2f}, y={kf_pose.position.y:.2f}")

                # Check for potential loop closures against older keyframes
                if len(self.keyframes) > self.loop_closure_min_keyframes:
                    current_kf_pos = kf_pose.position
                    # Compare with all keyframes except the last N
                    for past_kf_id, past_kf_pose in self.keyframes[:-self.loop_closure_min_keyframes]:
                        past_kf_pos = past_kf_pose.position
                        dist_sq = (current_kf_pos.x - past_kf_pos.x)**2 + (current_kf_pos.y - past_kf_pos.y)**2
                        if dist_sq < self.loop_closure_dist_thresh**2:
                            rospy.logwarn(f"[{rospy.get_name()}] *** POTENTIAL LOOP CLOSURE DETECTED *** "
                                          f"between keyframe {kf_id} (current) and keyframe {past_kf_id} (past). "
                                          f"Distance: {math.sqrt(dist_sq):.3f}m")
                            # In a full system, trigger geometric verification here


    def lines_callback(self, msg: LaneLineArray):
        """ Processes detected lines and updates the map grid. """
        if self.current_odom_pose is None:
            rospy.logwarn_throttle(5.0, f"[{rospy.get_name()}] Received lines but no odometry pose yet. Skipping map update.")
            return

        # Get current robot pose in the odom frame
        robot_x = self.current_odom_pose.position.x
        robot_y = self.current_odom_pose.position.y
        q = self.current_odom_pose.orientation
        _, _, robot_yaw = tf_trans.euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Process each detected line
        for line in msg.lines:
            # Line parameters are relative to the robot's frame (x-forward, y-left)
            line_angle_rel = line.angle
            line_dist_rel = line.distance

            # Calculate two points on the line in the ROBOT frame
            # Point 1: Closest point on the line to the robot origin
            # Angle of the normal vector pointing from origin to line
            normal_angle = line_angle_rel + math.pi/2.0 # Check convention
            # Correct normal angle calculation based on line angle relative to x-axis
            # If line angle is alpha, normal is alpha +/- pi/2
            # Let's find the point on the line closest to origin (0,0) in robot frame
            # This point is at distance d along the normal vector
            # Need consistent normal direction. Let's assume normal points "left" relative to line direction if angle is positive slope-like
            # Or simpler: find a point ON the line and use the angle.
            # Point on line: (d/sin(angle), 0) or (0, -d/cos(angle))? No.
            # Parametric form: x = t*cos(angle), y = t*sin(angle). No, that's line THROUGH origin.

            # Let's use the angle and distance directly to find endpoints of a segment
            # centered around the point perpendicular to the robot's x-axis (approx)
            # Or find the intersection with a line perpendicular to the robot's x-axis at x=0?

            # --- Robust method: Find two points using angle and distance ---
            # 1. Find a point on the line. The point (d*cos(normal), d*sin(normal)) is NOT necessarily on the line.
            # The point (x,y) such that x*cos(normal) + y*sin(normal) = d IS on the line.
            # Let's find the point on the line closest to the robot origin (0,0).
            # This point P_closest has coordinates (d*cos(normal_angle), d*sin(normal_angle))
            # Let's recalculate normal angle properly from line angle relative to robot X axis
            # Line direction vector: (cos(line_angle_rel), sin(line_angle_rel))
            # Normal vector: (-sin(line_angle_rel), cos(line_angle_rel)) OR (sin(line_angle_rel), -cos(line_angle_rel))
            # Let's use the first normal: points roughly "left" for positive slope lines
            norm_x = -math.sin(line_angle_rel)
            norm_y = math.cos(line_angle_rel)
            closest_pt_x_rob = line_dist_rel * norm_x
            closest_pt_y_rob = line_dist_rel * norm_y

            # 2. Define a segment along the line direction centered at this closest point
            line_dir_x = math.cos(line_angle_rel)
            line_dir_y = math.sin(line_angle_rel)
            half_len = self.line_segment_length / 2.0

            start_x_rob = closest_pt_x_rob - half_len * line_dir_x
            start_y_rob = closest_pt_y_rob - half_len * line_dir_y
            end_x_rob = closest_pt_x_rob + half_len * line_dir_x
            end_y_rob = closest_pt_y_rob + half_len * line_dir_y

            # Transform segment endpoints from robot frame to ODOM frame
            start_x_odom = robot_x + start_x_rob * math.cos(robot_yaw) - start_y_rob * math.sin(robot_yaw)
            start_y_odom = robot_y + start_x_rob * math.sin(robot_yaw) + start_y_rob * math.cos(robot_yaw)
            end_x_odom = robot_x + end_x_rob * math.cos(robot_yaw) - end_y_rob * math.sin(robot_yaw)
            end_y_odom = robot_y + end_x_rob * math.sin(robot_yaw) + end_y_rob * math.cos(robot_yaw)

            # Convert odom coordinates to map cell coordinates
            start_cell_x, start_cell_y = self.world_to_map_coords(start_x_odom, start_y_odom)
            end_cell_x, end_cell_y = self.world_to_map_coords(end_x_odom, end_y_odom)

            # Draw the line on the map grid if endpoints are valid
            if start_cell_x is not None and end_cell_x is not None:
                rr, cc = draw_line(start_cell_y, start_cell_x, end_cell_y, end_cell_x)
                # Ensure indices are within bounds before drawing
                valid_indices = (rr >= 0) & (rr < self.map_height_cells) & (cc >= 0) & (cc < self.map_width_cells)
                rr_valid = rr[valid_indices]
                cc_valid = cc[valid_indices]

                occ_value = self.value_occupied_white if line.color == LaneLine.COLOR_WHITE else self.value_occupied_yellow
                self.map_data[rr_valid, cc_valid] = occ_value


    def world_to_map_coords(self, world_x, world_y):
        """ Converts world coordinates (odom frame) to map cell coordinates. """
        if self.map_resolution == 0: return None, None
        # Translate world origin to map origin, then scale by resolution
        map_x = int(round((world_x - self.map_origin_x) / self.map_resolution))
        map_y = int(round((world_y - self.map_origin_y) / self.map_resolution))
        # Check if within map bounds
        if 0 <= map_x < self.map_width_cells and 0 <= map_y < self.map_height_cells:
            return map_x, map_y
        else:
            # Optional: Log if points fall outside map
            # rospy.logdebug_throttle(5.0, f"[{rospy.get_name()}] World point ({world_x:.2f}, {world_y:.2f}) is outside map bounds.")
            return None, None


    def publish_map(self, event=None):
        """ Publishes the current occupancy grid map. """
        map_msg = OccupancyGrid()
        map_msg.header.stamp = rospy.Time.now()
        map_msg.header.frame_id = self.map_frame_id # Should be "map"
        map_msg.info.map_load_time = map_msg.header.stamp # Use current time
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width_cells
        map_msg.info.height = self.map_height_cells
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y
        map_msg.info.origin.position.z = self.map_origin_z
        # Map orientation is typically identity
        map_msg.info.origin.orientation.x = 0.0
        map_msg.info.origin.orientation.y = 0.0
        map_msg.info.origin.orientation.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        # Flatten the map data (row-major order)
        map_msg.data = self.map_data.ravel().tolist()

        self.map_pub.publish(map_msg)
        rospy.logdebug(f"[{rospy.get_name()}] Published map update.")


if __name__ == '__main__':
    try:
        mapper = FeatureMapperNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Feature mapper node shut down.")
    except Exception as main_e:
        rospy.logfatal(f"Unhandled exception in feature mapper main: {main_e}")


