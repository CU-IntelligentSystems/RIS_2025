#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Description: Performs pose-graph SLAM backend optimization.
#              - Subscribes to filtered odometry and detected line features.
#              - Builds a pose graph using GTSAM (iSAM2).
#              - Detects loop closures based on pose proximity.
#              - Verifies loop closures by aligning line features using ICP.
#              - Publishes the map -> odom TF transform.

import rospy
import numpy as np
import math
import tf
import tf.transformations as tf_trans
from collections import deque

# ROS Msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
# Custom Msgs (Make sure your package is sourced)
try:
    # Replace 'slam_loopclosure' with your actual package name if different
    from slam_loopclosure.msg import LaneLine, LaneLineArray
except ImportError:
    rospy.logfatal("Could not import LaneLine messages. "
                   "Make sure the package is built and sourced, "
                   "and replace 'slam_loopclosure' with the correct package name.")
    exit()

# GTSAM
try:
    import gtsam
    from gtsam import Pose2, Rot2, Point2 # For 2D SLAM
    from gtsam import PriorFactorPose2, BetweenFactorPose2
    from gtsam import NonlinearFactorGraph, Values, ISAM2, ISAM2Params, Marginals
    from gtsam.symbol_shorthand import X # For pose variables X(i)
except ImportError:
    rospy.logfatal("GTSAM Python bindings not found. Install with 'pip install gtsam'")
    exit()

# Scikit-learn for ICP (NearestNeighbors)
try:
    from sklearn.neighbors import NearestNeighbors
except ImportError:
    rospy.logfatal("Scikit-learn not found. Install with 'pip install scikit-learn'")
    exit()


class PoseGraphOptimizerNode:
    def __init__(self):
        node_name = 'pose_graph_optimizer_node'
        rospy.init_node(node_name, anonymous=True)
        rospy.loginfo(f"[{node_name}] Initializing...")

        # --- Parameters ---
        self.odom_frame_id = rospy.get_param("~odom_frame_id", "odom")
        self.map_frame_id = rospy.get_param("~map_frame_id", "map")
        self.base_frame_id = rospy.get_param("~base_frame_id", "ente/base") # Used for context, not direct TF

        # Keyframe thresholds
        self.keyframe_dist_thresh = rospy.get_param("~keyframe_dist_thresh", 0.3) # meters
        self.keyframe_angle_thresh = rospy.get_param("~keyframe_angle_thresh", np.deg2rad(10)) # radians

        # Loop Closure thresholds
        self.lc_dist_thresh = rospy.get_param("~loop_closure_dist_thresh", 0.7) # meters (larger than keyframe)
        self.lc_min_keyframes_apart = rospy.get_param("~loop_closure_min_keyframes_apart", 20) # Min index difference
        self.lc_max_candidates = rospy.get_param("~loop_closure_max_candidates", 3) # Check top N proximity candidates

        # ICP parameters
        self.icp_max_iterations = rospy.get_param("~icp_max_iterations", 20)
        self.icp_tolerance = rospy.get_param("~icp_tolerance", 1e-3) # Convergence tolerance for transformation change
        self.icp_max_correspondence_dist = rospy.get_param("~icp_max_correspondence_dist", 0.2) # meters
        self.icp_min_inliers = rospy.get_param("~icp_min_inliers", 5) # Min points needed for valid match
        self.icp_line_sampling_dist = rospy.get_param("~icp_line_sampling_dist", 0.05) # meters between sampled points
        self.icp_line_segment_length = rospy.get_param("~icp_line_segment_length", 0.3) # Match length used in mapper

        # --- State Variables ---
        self.last_odom_pose = None      # Last received Odometry message
        self.last_lines_msg = None    # Last received LaneLineArray message
        self.last_keyframe_pose2 = None # Last gtsam.Pose2 added as keyframe
        self.keyframe_counter = 0       # ID for the next keyframe/pose node

        # Data associated with each keyframe
        self.keyframe_data = {} # Dict: keyframe_id -> {'pose': Pose2, 'lines': LaneLineArray, 'odom_pose': Pose}

        # GTSAM Graph and Values
        self.graph = NonlinearFactorGraph()
        self.initial_estimates = Values() # Store initial estimates for GTSAM
        self.current_estimates = None     # Store optimized results from iSAM2

        # iSAM2 optimizer
        isam_params = ISAM2Params()
        isam_params.setRelinearizeThreshold(0.1) # Default 0.1
        isam_params.setRelinearizeSkip(1)       # Default 1
        # isam_params.setFactorization("CHOLESKY") # QR (default) or CHOLESKY
        self.isam = ISAM2(isam_params)

        # TF Broadcaster for map -> odom
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.current_map_to_odom = None # Store the latest calculated transform

        # --- ROS Comms ---
        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback, queue_size=5)
        # Make sure the topic name matches where line_status_detector_updated publishes
        rospy.Subscriber("/line_parameter_detector/lines", LaneLineArray, self.lines_callback, queue_size=2)

        # Timer for periodic TF publishing
        # Optimization happens incrementally in odom_callback now
        self.tf_publish_timer = rospy.Timer(rospy.Duration(0.1), self.publish_tf) # Publish TF at 10Hz

        # --- Load Noise Models (Corrected Indentation) ---
        # Ensure these parameter names match your optimizer_params.yaml
        try:
            odom_s = rospy.get_param("~odom_noise_sigmas", [0.05, 0.05, np.deg2rad(2)])
            lc_s = rospy.get_param("~lc_noise_sigmas", [0.1, 0.1, np.deg2rad(5)])
            prior_s = rospy.get_param("~prior_noise_sigmas", [1e-6, 1e-6, 1e-6])

            self.prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array(prior_s))
            self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array(odom_s))
            self.lc_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array(lc_s))
            rospy.loginfo(f"[{node_name}] Noise models loaded: Prior={prior_s}, Odom={odom_s}, LC={lc_s}")
        except Exception as e:
            rospy.logfatal(f"[{node_name}] Error loading noise parameters: {e}")
            rospy.signal_shutdown("Failed to load noise parameters.")
            return

        rospy.loginfo(f"[{node_name}] Initialization complete.")
        rospy.loginfo(f"[{node_name}] Waiting for odometry and line data...")


    def odom_callback(self, msg: Odometry):
        """ Processes odometry, decides if it's a keyframe, adds factors. """
        self.last_odom_pose = msg

        # Convert ROS Odometry Pose to gtsam.Pose2
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = tf_trans.euler_from_quaternion([q.x, q.y, q.z, q.w])
        current_pose2_odom = Pose2(pos.x, pos.y, yaw) # Pose in odom frame

        # If first frame, add prior and initialize
        if self.keyframe_counter == 0:
            rospy.loginfo(f"[{rospy.get_name()}] Received first odometry. Adding prior at origin.")
            # Assume map frame starts aligned with odom frame initially
            initial_pose_map = Pose2(0.0, 0.0, 0.0)
            self.graph.add(PriorFactorPose2(X(0), initial_pose_map, self.prior_noise))
            self.initial_estimates.insert(X(0), initial_pose_map)
            # Store the initial odom pose corresponding to this keyframe
            self.keyframe_data[0] = {'pose_map_initial': initial_pose_map, 'lines': self.last_lines_msg, 'odom_pose': msg.pose.pose}
            self.last_keyframe_pose2_odom = current_pose2_odom # Store the odom pose
            self.keyframe_counter += 1
            # Perform initial update
            try:
                self.isam.update(self.graph, self.initial_estimates)
                self.current_estimates = self.isam.calculateEstimate()
                self.graph = NonlinearFactorGraph() # Clear graph and values for next incremental update
                self.initial_estimates = Values()
                rospy.loginfo(f"[{rospy.get_name()}] Initial optimization complete.")
            except Exception as e:
                rospy.logerr(f"[{rospy.get_name()}] Error during initial iSAM update: {e}")

            self.calculate_and_store_map_to_odom() # Calculate initial TF
            return

        # Keyframe decision (based on motion in odom frame)
        add_keyframe = False
        if self.last_keyframe_pose2_odom is None: # Should not happen after first frame
            rospy.logwarn_throttle(5.0, f"[{rospy.get_name()}] last_keyframe_pose2_odom is None after initialization!")
            return

        # Calculate delta pose in the odom frame
        delta_pose_odom = self.last_keyframe_pose2_odom.between(current_pose2_odom)
        dist = delta_pose_odom.translation().norm()
        angle = abs(delta_pose_odom.rotation().theta())

        if dist > self.keyframe_dist_thresh or angle > self.keyframe_angle_thresh:
            add_keyframe = True

        if add_keyframe:
            k_prev = self.keyframe_counter - 1
            k_curr = self.keyframe_counter

            rospy.logdebug(f"[{rospy.get_name()}] Adding keyframe {k_curr}. Dist={dist:.2f}, Angle={np.rad2deg(angle):.1f}")

            # Add odometry factor (using the delta calculated in odom frame)
            self.graph.add(BetweenFactorPose2(X(k_prev), X(k_curr), delta_pose_odom, self.odom_noise))

            # Add initial estimate for the new pose (relative to previous *optimized* pose)
            new_initial_estimate_map = Pose2() # Default constructor
            if self.current_estimates and self.current_estimates.exists(X(k_prev)):
                 prev_optimized_pose_map = self.current_estimates.atPose2(X(k_prev))
                 # Apply the odom delta to the previous optimized map pose
                 new_initial_estimate_map = prev_optimized_pose_map.compose(delta_pose_odom)
            else:
                 # Fallback if previous estimate doesn't exist (should only happen if k_prev=0 failed)
                 # Estimate based on current odom pose relative to initial odom pose
                 initial_odom_pose = self.keyframe_data[0]['odom_pose']
                 initial_odom_pose2 = Pose2(initial_odom_pose.position.x, initial_odom_pose.position.y,
                                            tf_trans.euler_from_quaternion([initial_odom_pose.orientation.x, initial_odom_pose.orientation.y, initial_odom_pose.orientation.z, initial_odom_pose.orientation.w])[2])
                 pose_relative_to_start_odom = initial_odom_pose2.between(current_pose2_odom)
                 # Assume map started aligned with odom
                 new_initial_estimate_map = self.keyframe_data[0]['pose_map_initial'].compose(pose_relative_to_start_odom)
                 rospy.logwarn(f"[{rospy.get_name()}] Fallback initial estimate used for KF {k_curr}")

            self.initial_estimates.insert(X(k_curr), new_initial_estimate_map)

            # Store data associated with this keyframe
            self.keyframe_data[k_curr] = {'pose_map_initial': new_initial_estimate_map, 'lines': self.last_lines_msg, 'odom_pose': msg.pose.pose}
            self.last_keyframe_pose2_odom = current_pose2_odom # Update last odom pose used for keyframe checks
            self.keyframe_counter += 1

            # Attempt Loop Closure
            self.attempt_loop_closure(k_curr, new_initial_estimate_map)

            # --- Incremental Update ---
            try:
                self.isam.update(self.graph, self.initial_estimates)
                self.current_estimates = self.isam.calculateEstimate()
                self.graph = NonlinearFactorGraph() # Clear factors processed by isam
                self.initial_estimates = Values() # Clear initial estimates used by isam
                self.calculate_and_store_map_to_odom() # Update TF after optimization
            except Exception as e:
                 # Use %s for exception formatting
                 rospy.logerr(f"[{rospy.get_name()}] Error during iSAM update for keyframe {k_curr}: %s", e)


    def lines_callback(self, msg: LaneLineArray):
        """ Stores the latest line detection results. """
        self.last_lines_msg = msg


    def attempt_loop_closure(self, current_kf_id, current_kf_pose_estimate_map):
        """ Finds potential loop closure candidates and tries to verify them. """
        if self.keyframe_counter < self.lc_min_keyframes_apart + 2:
            return # Not enough history to find loops

        candidates = []
        # Find potential candidates based on proximity of estimated poses in MAP frame
        for past_kf_id in range(current_kf_id - self.lc_min_keyframes_apart):
            if past_kf_id not in self.keyframe_data: continue

            # Use current best estimates for proximity check
            past_kf_pose_estimate_map = Pose2() # Default
            if self.current_estimates and self.current_estimates.exists(X(past_kf_id)):
                past_kf_pose_estimate_map = self.current_estimates.atPose2(X(past_kf_id))
            else:
                # Fallback to initial estimate if optimized one not available (less likely now)
                past_kf_pose_estimate_map = self.keyframe_data[past_kf_id]['pose_map_initial']

            # Calculate distance in the map frame
            dist = current_kf_pose_estimate_map.translation().distance(past_kf_pose_estimate_map.translation())

            if dist < self.lc_dist_thresh:
                candidates.append({'id': past_kf_id, 'dist': dist})

        if not candidates:
            return

        # Sort candidates by distance and take the top N
        candidates.sort(key=lambda c: c['dist'])
        rospy.loginfo(f"[{rospy.get_name()}] Found {len(candidates)} loop closure candidates for KF {current_kf_id}.")

        added_lc = False
        for candidate in candidates[:self.lc_max_candidates]:
            past_kf_id = candidate['id']
            rospy.loginfo(f"[{rospy.get_name()}] Verifying LC candidate: {current_kf_id} <-> {past_kf_id} (Est. Map Dist: {candidate['dist']:.2f}m)")

            # --- Geometric Verification using ICP on Line Features ---
            # Pass the current best estimates of poses in the map frame to ICP
            transform_matrix_map, success = self.verify_lines_icp(current_kf_id, past_kf_id)

            if success:
                rospy.logwarn(f"[{rospy.get_name()}] *** LOOP CLOSURE VERIFIED *** Adding factor between {past_kf_id} and {current_kf_id}")
                # The transform_matrix_map is T_past_current in the map frame (4x4)
                # Convert it to Pose2 relative transform
                relative_pose_map = Pose2(transform_matrix_map[0, 3], transform_matrix_map[1, 3], math.atan2(transform_matrix_map[1, 0], transform_matrix_map[0, 0]))

                # Add the loop closure factor to the graph
                self.graph.add(BetweenFactorPose2(X(past_kf_id), X(current_kf_id), relative_pose_map, self.lc_noise))
                added_lc = True
                # Optional: Break after finding the first good loop closure
                # break
            else:
                 rospy.loginfo(f"[{rospy.get_name()}] LC verification FAILED between {current_kf_id} and {past_kf_id}.")

        # If loop closures were added, the next call to isam.update() in odom_callback will process them.


    def verify_lines_icp(self, kf_id_curr, kf_id_past):
        """
        Performs ICP alignment between line features of two keyframes.
        Uses current estimated poses from the graph.
        Returns: (4x4 Transformation Matrix T_past_current in map frame, success_flag)
                 Returns (None, False) on failure.
        """
        if kf_id_curr not in self.keyframe_data or kf_id_past not in self.keyframe_data:
            rospy.logwarn(f"[{rospy.get_name()}] Missing keyframe data for ICP: {kf_id_curr} or {kf_id_past}")
            return None, False

        lines_curr = self.keyframe_data[kf_id_curr]['lines']
        lines_past = self.keyframe_data[kf_id_past]['lines']

        if lines_curr is None or lines_past is None or not lines_curr.lines or not lines_past.lines:
            rospy.logdebug(f"[{rospy.get_name()}] Not enough line data for ICP between {kf_id_curr} and {kf_id_past}")
            return None, False

        # Get current estimates of the poses IN THE MAP FRAME
        pose_curr_map = Pose2()
        if self.current_estimates and self.current_estimates.exists(X(kf_id_curr)):
            pose_curr_map = self.current_estimates.atPose2(X(kf_id_curr))
        else: pose_curr_map = self.keyframe_data[kf_id_curr]['pose_map_initial'] # Fallback

        pose_past_map = Pose2()
        if self.current_estimates and self.current_estimates.exists(X(kf_id_past)):
            pose_past_map = self.current_estimates.atPose2(X(kf_id_past))
        else: pose_past_map = self.keyframe_data[kf_id_past]['pose_map_initial'] # Fallback


        # 1. Generate Point Clouds from Lines in the MAP frame
        pc_curr_map_frame = self.generate_pointcloud_from_lines(lines_curr, pose_curr_map)
        pc_past_map_frame = self.generate_pointcloud_from_lines(lines_past, pose_past_map)

        if pc_curr_map_frame.shape[0] < self.icp_min_inliers or pc_past_map_frame.shape[0] < self.icp_min_inliers:
             rospy.logdebug(f"[{rospy.get_name()}] Not enough points generated for ICP ({pc_curr_map_frame.shape[0]} vs {pc_past_map_frame.shape[0]})")
             return None, False

        # 2. Basic Point-to-Point ICP Implementation
        # Source cloud: pc_past_map_frame (will be transformed) - Points already in map frame
        # Target cloud: pc_curr_map_frame (fixed reference) - Points already in map frame
        src = pc_past_map_frame.T # Shape (2, N_past)
        dst = pc_curr_map_frame.T # Shape (2, N_curr)

        # Initial guess for transformation (relative transform between estimated map poses)
        T_curr_past_initial_map = pose_curr_map.between(pose_past_map) # Transform FROM curr TO past in map frame
        T_past_curr_initial_map = T_curr_past_initial_map.inverse() # Transform FROM past TO curr in map frame

        R_init = T_past_curr_initial_map.rotation().matrix() # 2x2 rotation
        t_init = T_past_curr_initial_map.translation().vector() # 2x1 translation
        T_icp = np.identity(3) # Homogeneous transform (2D) for ICP calculation
        T_icp[:2, :2] = R_init
        T_icp[:2, 2] = t_init.flatten() # Ensure t_init is flat

        src_transformed = T_icp[:2, :2] @ src + T_icp[:2, 2, np.newaxis] # Apply initial transform

        # Use KDTree for nearest neighbor search
        nn = NearestNeighbors(n_neighbors=1, algorithm='kd_tree')
        nn.fit(dst.T) # Fit KD-Tree to target points (already in map frame)

        prev_mean_error = float('inf')

        for i in range(self.icp_max_iterations):
            # Find correspondences: For each point in transformed source, find nearest in target
            distances, indices = nn.kneighbors(src_transformed.T)
            distances = distances.ravel()
            indices = indices.ravel()

            # Filter correspondences based on distance threshold
            valid_mask = distances < self.icp_max_correspondence_dist
            num_inliers = np.sum(valid_mask)

            if num_inliers < self.icp_min_inliers:
                rospy.logdebug(f"[{rospy.get_name()}] ICP failed: Not enough inliers ({num_inliers}) at iter {i}")
                return None, False

            # Get corresponding points (use original source points for calculation)
            src_corr = src[:, valid_mask]
            dst_corr = dst[:, indices[valid_mask]]

            # Calculate transformation update using SVD (Arun's method/Kabsch algorithm)
            centroid_src = np.mean(src_corr, axis=1, keepdims=True)
            centroid_dst = np.mean(dst_corr, axis=1, keepdims=True)
            centered_src = src_corr - centroid_src
            centered_dst = dst_corr - centroid_dst
            H = centered_src @ centered_dst.T # Covariance matrix H (2x2)
            U, S, Vt = np.linalg.svd(H)     # Use SVD on H
            R_update = Vt.T @ U.T           # Calculate optimal rotation R

            # Handle potential reflection case (det(R) = -1)
            if np.linalg.det(R_update) < 0:
                rospy.logdebug(f"[{rospy.get_name()}] ICP reflection detected, correcting...")
                Vt[1, :] *= -1 # Flip the sign of the last row of Vt
                R_update = Vt.T @ U.T # Recalculate R

            t_update = centroid_dst - R_update @ centroid_src # Calculate optimal translation t

            # Create the incremental homogeneous transformation matrix
            T_update = np.identity(3)
            T_update[:2, :2] = R_update
            T_update[:2, 2] = t_update.flatten()

            # Update the total transformation (applied to original source points)
            T_icp = T_update @ T_icp

            # Apply the updated total transform to the original source points for next iteration's NN search
            src_transformed = T_icp[:2, :2] @ src + T_icp[:2, 2, np.newaxis]

            # Calculate mean error for convergence check
            new_distances, _ = nn.kneighbors(src_transformed.T)
            mean_error = np.mean(new_distances[new_distances < self.icp_max_correspondence_dist]) # Error based on inliers

            # Check for convergence (change in mean error)
            if abs(prev_mean_error - mean_error) < self.icp_tolerance:
                 rospy.loginfo(f"[{rospy.get_name()}] ICP converged at iter {i+1} with error {mean_error:.4f} ({num_inliers} inliers)")
                 # Convert final 3x3 homogeneous 2D transform T_icp to 4x4 homogeneous 3D
                 # This T_icp represents the transform FROM the past cloud's frame TO the current cloud's frame (map frame)
                 T_final_4x4 = np.identity(4)
                 T_final_4x4[0:2, 0:2] = T_icp[0:2, 0:2] # Copy R (2x2)
                 T_final_4x4[0:2, 3] = T_icp[0:2, 2]   # Copy t (2x1) into x, y translation
                 return T_final_4x4, True # Success

            prev_mean_error = mean_error

        rospy.logwarn(f"[{rospy.get_name()}] ICP failed: Max iterations ({self.icp_max_iterations}) reached without convergence.")
        return None, False


    def generate_pointcloud_from_lines(self, lines_msg: LaneLineArray, pose_map: Pose2):
        """ Generates a 2D point cloud (Nx2 NumPy array) in the MAP frame from LaneLine messages. """
        points_map = []
        if lines_msg is None: return np.empty((0, 2))

        # Transform matrix from robot frame (at pose_map) to map frame
        R_map_robot = pose_map.rotation().matrix() # 2x2
        t_map_robot = pose_map.translation().vector() # 2x1

        for line in lines_msg.lines:
            # Reconstruct line segment in ROBOT frame (as done in mapper)
            line_angle_rel = line.angle
            line_dist_rel = line.distance
            # Calculate normal vector components in robot frame
            norm_x_rob = -math.sin(line_angle_rel)
            norm_y_rob = math.cos(line_angle_rel)
            # Calculate closest point on line to robot origin in robot frame
            closest_pt_x_rob = line_dist_rel * norm_x_rob
            closest_pt_y_rob = line_dist_rel * norm_y_rob
            # Calculate line direction vector components in robot frame
            line_dir_x_rob = math.cos(line_angle_rel)
            line_dir_y_rob = math.sin(line_angle_rel)
            # Calculate segment endpoints in robot frame
            half_len = self.icp_line_segment_length / 2.0
            start_x_rob = closest_pt_x_rob - half_len * line_dir_x_rob
            start_y_rob = closest_pt_y_rob - half_len * line_dir_y_rob
            end_x_rob = closest_pt_x_rob + half_len * line_dir_x_rob
            end_y_rob = closest_pt_y_rob + half_len * line_dir_y_rob

            # Sample points along the segment in ROBOT frame
            num_samples = int(math.ceil(self.icp_line_segment_length / self.icp_line_sampling_dist)) + 1
            if num_samples < 2: num_samples = 2
            x_samples_rob = np.linspace(start_x_rob, end_x_rob, num_samples)
            y_samples_rob = np.linspace(start_y_rob, end_y_rob, num_samples)
            points_robot = np.vstack((x_samples_rob, y_samples_rob)) # Shape (2, num_samples)

            # Transform points to MAP frame: p_map = R_map_robot * p_robot + t_map_robot
            points_tf_map = R_map_robot @ points_robot + t_map_robot[:, np.newaxis]
            points_map.append(points_tf_map.T) # Append as (num_samples, 2)

        if not points_map:
            return np.empty((0, 2))
        return np.concatenate(points_map, axis=0)


    def calculate_and_store_map_to_odom(self):
        """ Calculates the map -> odom transform based on latest optimized pose. """
        if self.current_estimates is None or self.keyframe_counter == 0:
            return

        latest_kf_id = self.keyframe_counter - 1
        if not self.current_estimates.exists(X(latest_kf_id)):
             rospy.logwarn_throttle(5.0, f"[{rospy.get_name()}] Cannot calculate TF: Latest keyframe {latest_kf_id} not in estimates.")
             return

        # Get the optimized pose of the latest keyframe in the map frame
        pose_map_latest_kf = self.current_estimates.atPose2(X(latest_kf_id))

        # Get the corresponding original odometry pose for that keyframe (in odom frame)
        if latest_kf_id not in self.keyframe_data:
            rospy.logwarn_throttle(5.0, f"[{rospy.get_name()}] Cannot calculate TF: Keyframe data missing for {latest_kf_id}.")
            return
        # Retrieve the ROS Pose message stored when the keyframe was created
        odom_pose_latest_kf_ros = self.keyframe_data[latest_kf_id]['odom_pose']
        pos = odom_pose_latest_kf_ros.position
        q = odom_pose_latest_kf_ros.orientation
        _, _, yaw = tf_trans.euler_from_quaternion([q.x, q.y, q.z, q.w])
        pose_odom_latest_kf = Pose2(pos.x, pos.y, yaw) # Pose of latest KF in odom frame

        # Calculate the transform: T_map_odom = T_map_kf * T_kf_odom
        # Where T_kf_odom = T_odom_kf.inverse()
        pose_kf_odom = pose_odom_latest_kf.inverse()
        map_to_odom_transform = pose_map_latest_kf.compose(pose_kf_odom)

        self.current_map_to_odom = map_to_odom_transform
        rospy.logdebug(f"[{rospy.get_name()}] Updated map->odom TF: x={map_to_odom_transform.x():.3f}, y={map_to_odom_transform.y():.3f}, th={map_to_odom_transform.theta():.3f}")


    def publish_tf(self, event=None):
        """ Periodically publishes the calculated map -> odom TF. """
        if self.current_map_to_odom is not None:
            t = self.current_map_to_odom.translation()
            r = self.current_map_to_odom.rotation().theta()
            q_tf = tf_trans.quaternion_from_euler(0, 0, r)
            current_time = rospy.Time.now()

            try:
                self.tf_broadcaster.sendTransform(
                    (t.x(), t.y(), 0.0), # translation
                    q_tf,               # rotation
                    current_time,       # timestamp
                    self.odom_frame_id, # child frame
                    self.map_frame_id   # parent frame
                )
            except Exception as e:
                 # Use %s for exception formatting
                 rospy.logerr_throttle(5.0, f"[{rospy.get_name()}] Error broadcasting TF: %s", e)


if __name__ == '__main__':
    try:
        node = PoseGraphOptimizerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Pose graph optimizer node shut down.")
    except Exception as main_e:
        # Use %s for exception formatting
        rospy.logfatal(f"Unhandled exception in optimizer main: %s", main_e)
        import traceback
        traceback.print_exc() # Print detailed traceback


