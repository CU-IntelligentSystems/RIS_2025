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

try:
    from slam_loopclosure.msg import LaneLine, LaneLineArray
except ImportError:
    rospy.logfatal("Could not import LaneLine messages. "
                   "Make sure the package is built and sourced, "
                  )
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

        # Noise models (Tune these based on sensor/odometry performance!)
        # Prior on the first pose (very certain)
        prior_sigmas = np.array([1e-6, 1e-6, 1e-6]) # x, y, theta
        self.prior_noise = gtsam.noiseModel.Diagonal.Sigmas(prior_sigmas)
        # Odometry noise (less certain) - Should reflect drift between keyframes
        odom_sigmas = np.array([0.05, 0.05, np.deg2rad(2)]) # x, y, theta per keyframe step (example)
        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(odom_sigmas)
        # Loop closure noise (uncertainty of the ICP alignment) - Start relatively high
        lc_sigmas = np.array([0.1, 0.1, np.deg2rad(5)]) # x, y, theta (example)
        self.lc_noise = gtsam.noiseModel.Diagonal.Sigmas(lc_sigmas)


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
        rospy.Subscriber("/line_parameter_detector/lines", LaneLineArray, self.lines_callback, queue_size=2)

        # Timer for periodic optimization and TF publishing
        self.optimize_timer = rospy.Timer(rospy.Duration(1.0), self.optimize_and_publish_tf) # Optimize every second

        rospy.loginfo(f"[{node_name}] Initialization complete.")
        rospy.loginfo(f"[{node_name}] Waiting for odometry and line data...")


    def odom_callback(self, msg: Odometry):
        """ Processes odometry, decides if it's a keyframe, adds factors. """
        self.last_odom_pose = msg

        # Convert ROS Odometry Pose to gtsam.Pose2
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = tf_trans.euler_from_quaternion([q.x, q.y, q.z, q.w])
        current_pose2 = Pose2(pos.x, pos.y, yaw)

        # If first frame, add prior and initialize
        if self.keyframe_counter == 0:
            rospy.loginfo(f"[{rospy.get_name()}] Received first odometry. Adding prior.")
            self.graph.add(PriorFactorPose2(X(0), current_pose2, self.prior_noise))
            self.initial_estimates.insert(X(0), current_pose2)
            self.keyframe_data[0] = {'pose': current_pose2, 'lines': self.last_lines_msg, 'odom_pose': msg.pose.pose}
            self.last_keyframe_pose2 = current_pose2
            self.keyframe_counter += 1
            # Perform initial update
            try:
                self.isam.update(self.graph, self.initial_estimates)
                self.current_estimates = self.isam.calculateEstimate()
                # Clear graph and values for next incremental update
                self.graph = NonlinearFactorGraph()
                self.initial_estimates = Values()
            except Exception as e:
                rospy.logerr(f"[{rospy.get_name()}] Error during initial iSAM update: {e}")

            self.calculate_and_store_map_to_odom() # Calculate initial TF
            return

        # Keyframe decision
        add_keyframe = False
        if self.last_keyframe_pose2 is None: # Should not happen after first frame
             rospy.logwarn_throttle(5.0, f"[{rospy.get_name()}] last_keyframe_pose2 is None after initialization!")
             return

        delta_pose = self.last_keyframe_pose2.between(current_pose2)
        dist = delta_pose.translation().norm()
        angle = abs(delta_pose.rotation().theta())

        if dist > self.keyframe_dist_thresh or angle > self.keyframe_angle_thresh:
            add_keyframe = True

        if add_keyframe:
            k_prev = self.keyframe_counter - 1
            k_curr = self.keyframe_counter

            rospy.logdebug(f"[{rospy.get_name()}] Adding keyframe {k_curr}. Dist={dist:.2f}, Angle={np.rad2deg(angle):.1f}")

            # Add odometry factor
            self.graph.add(BetweenFactorPose2(X(k_prev), X(k_curr), delta_pose, self.odom_noise))
            # Add initial estimate for the new pose (relative to previous optimized pose)
            if self.current_estimates and self.current_estimates.exists(X(k_prev)):
                 prev_optimized_pose = self.current_estimates.atPose2(X(k_prev))
                 new_initial_estimate = prev_optimized_pose.compose(delta_pose)
            else:
                 # Fallback if previous estimate doesn't exist (shouldn't happen often)
                 new_initial_estimate = current_pose2 # Use odom directly
            self.initial_estimates.insert(X(k_curr), new_initial_estimate)

            # Store data associated with this keyframe
            self.keyframe_data[k_curr] = {'pose': new_initial_estimate, 'lines': self.last_lines_msg, 'odom_pose': msg.pose.pose}
            self.last_keyframe_pose2 = current_pose2 # Update last odom pose used for keyframe checks
            self.keyframe_counter += 1

            # Attempt Loop Closure
            self.attempt_loop_closure(k_curr, new_initial_estimate)

            # --- Incremental Update (Can be done here or in timer) ---
            # Doing it here ensures graph is updated immediately after adding factors
            try:
                self.isam.update(self.graph, self.initial_estimates)
                self.current_estimates = self.isam.calculateEstimate()
                self.graph = NonlinearFactorGraph() # Clear factors processed by isam
                self.initial_estimates = Values() # Clear initial estimates used by isam
                self.calculate_and_store_map_to_odom() # Update TF after optimization
            except Exception as e:
                 rospy.logerr(f"[{rospy.get_name()}] Error during iSAM update for keyframe {k_curr}: {e}")


    def lines_callback(self, msg: LaneLineArray):
        """ Stores the latest line detection results. """
        self.last_lines_msg = msg


    def attempt_loop_closure(self, current_kf_id, current_kf_pose_estimate):
        """ Finds potential loop closure candidates and tries to verify them. """
        if self.keyframe_counter < self.lc_min_keyframes_apart + 2:
            return # Not enough history to find loops

        candidates = []
        # Find potential candidates based on proximity of estimated poses
        for past_kf_id in range(current_kf_id - self.lc_min_keyframes_apart):
            if past_kf_id not in self.keyframe_data: continue

            # Use current best estimates for proximity check
            if self.current_estimates and self.current_estimates.exists(X(past_kf_id)):
                past_kf_pose_estimate = self.current_estimates.atPose2(X(past_kf_id))
            else:
                # Fallback to initial estimate if optimized one not available
                past_kf_pose_estimate = self.keyframe_data[past_kf_id]['pose']

            dist = current_kf_pose_estimate.translation().distance(past_kf_pose_estimate.translation())

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
            rospy.loginfo(f"[{rospy.get_name()}] Verifying LC candidate: {current_kf_id} <-> {past_kf_id} (Dist: {candidate['dist']:.2f}m)")

            # --- Geometric Verification using ICP on Line Features ---
            transform, success = self.verify_lines_icp(current_kf_id, past_kf_id)

            if success:
                rospy.logwarn(f"[{rospy.get_name()}] *** LOOP CLOSURE VERIFIED *** Adding factor between {past_kf_id} and {current_kf_id}")
                # Add the loop closure factor to the graph
                # The transform is from past_kf frame to current_kf frame (estimated by ICP)
                # We need the transform FROM past TO current for BetweenFactor
                # Check ICP output convention carefully! Assuming T_past_current
                relative_pose = Pose2(transform[0, 2], transform[1, 2], math.atan2(transform[1, 0], transform[0, 0]))
                self.graph.add(BetweenFactorPose2(X(past_kf_id), X(current_kf_id), relative_pose, self.lc_noise))
                added_lc = True
                # Optional: Break after finding the first good loop closure
                # break
            else:
                 rospy.loginfo(f"[{rospy.get_name()}] LC verification FAILED between {current_kf_id} and {past_kf_id}.")

        # If loop closures were added, trigger an optimization immediately
        # The optimization might already be happening in odom_callback after this function returns
        # Or trigger it explicitly if optimization is only done in the timer
        # if added_lc:
        #     self.optimize_and_publish_tf() # Or just let the timer handle it


    def verify_lines_icp(self, kf_id_curr, kf_id_past):
        """
        Performs ICP alignment between line features of two keyframes.
        Returns: (4x4 Transformation Matrix T_past_current, success_flag)
                 Returns (None, False) on failure.
        Note: This is a basic point-to-point ICP implementation.
        """
        if kf_id_curr not in self.keyframe_data or kf_id_past not in self.keyframe_data:
            rospy.logwarn(f"[{rospy.get_name()}] Missing keyframe data for ICP: {kf_id_curr} or {kf_id_past}")
            return None, False

        lines_curr = self.keyframe_data[kf_id_curr]['lines']
        lines_past = self.keyframe_data[kf_id_past]['lines']

        if lines_curr is None or lines_past is None or not lines_curr.lines or not lines_past.lines:
            rospy.logdebug(f"[{rospy.get_name()}] Not enough line data for ICP between {kf_id_curr} and {kf_id_past}")
            return None, False

        # Get current estimates of the poses
        if self.current_estimates and self.current_estimates.exists(X(kf_id_curr)):
            pose_curr_map = self.current_estimates.atPose2(X(kf_id_curr))
        else: pose_curr_map = self.keyframe_data[kf_id_curr]['pose']

        if self.current_estimates and self.current_estimates.exists(X(kf_id_past)):
            pose_past_map = self.current_estimates.atPose2(X(kf_id_past))
        else: pose_past_map = self.keyframe_data[kf_id_past]['pose']


        # 1. Generate Point Clouds from Lines in their respective MAP frames
        pc_curr_map = self.generate_pointcloud_from_lines(lines_curr, pose_curr_map)
        pc_past_map = self.generate_pointcloud_from_lines(lines_past, pose_past_map)

        if pc_curr_map.shape[0] < self.icp_min_inliers or pc_past_map.shape[0] < self.icp_min_inliers:
             rospy.logdebug(f"[{rospy.get_name()}] Not enough points generated for ICP ({pc_curr_map.shape[0]} vs {pc_past_map.shape[0]})")
             return None, False

        # 2. Basic Point-to-Point ICP Implementation
        # Source cloud: pc_past_map (will be transformed)
        # Target cloud: pc_curr_map (fixed reference)
        src = pc_past_map.T # Shape (2, N_past)
        dst = pc_curr_map.T # Shape (2, N_curr)

        # Initial guess for transformation (from past map frame to current map frame)
        # T_map_curr = pose_curr_map
        # T_map_past = pose_past_map
        # T_curr_map = T_map_curr.inverse()
        # T_curr_past = T_curr_map * T_map_past
        T_curr_past_initial = pose_curr_map.between(pose_past_map) # Transform FROM curr TO past
        T_past_curr_initial = T_curr_past_initial.inverse() # Transform FROM past TO curr (ICP needs this)

        R_init = T_past_curr_initial.rotation().matrix() # 2x2 rotation
        t_init = T_past_curr_initial.translation().vector() # 2x1 translation
        T_icp = np.identity(3) # Homogeneous transform (2D)
        T_icp[:2, :2] = R_init
        T_icp[:2, 2] = t_init

        src_transformed = T_icp[:2, :2] @ src + T_icp[:2, 2, np.newaxis] # Apply initial transform

        nn = NearestNeighbors(n_neighbors=1, algorithm='kd_tree')
        nn.fit(dst.T) # Fit KD-Tree to target points

        prev_error = float('inf')

        for i in range(self.icp_max_iterations):
            # Find correspondences
            distances, indices = nn.kneighbors(src_transformed.T)
            distances = distances.ravel()
            indices = indices.ravel()

            # Filter correspondences based on distance
            valid_mask = distances < self.icp_max_correspondence_dist
            if np.sum(valid_mask) < self.icp_min_inliers:
                rospy.logdebug(f"[{rospy.get_name()}] ICP failed: Not enough inliers ({np.sum(valid_mask)}) at iter {i}")
                return None, False

            src_corr = src[:, valid_mask] # Corresponding points in original source cloud
            dst_corr = dst[:, indices[valid_mask]] # Corresponding points in target cloud

            # Calculate transformation using SVD (Procrustes analysis / Arun's method)
            centroid_src = np.mean(src_corr, axis=1, keepdims=True)
            centroid_dst = np.mean(dst_corr, axis=1, keepdims=True)
            centered_src = src_corr - centroid_src
            centered_dst = dst_corr - centroid_dst
            H = centered_src @ centered_dst.T # Covariance matrix (2x2)
            U, _, Vt = np.linalg.svd(H)
            R_update = Vt.T @ U.T # Rotation matrix (2x2)

            # Handle reflection case (det(R) = -1)
            if np.linalg.det(R_update) < 0:
                Vt[1, :] *= -1
                R_update = Vt.T @ U.T

            t_update = centroid_dst - R_update @ centroid_src # Translation vector (2x1)

            # Compose the update with the current total transform
            T_update = np.identity(3)
            T_update[:2, :2] = R_update
            T_update[:2, 2] = t_update.flatten()
            T_icp = T_update @ T_icp # Update total transform

            # Apply new transform and calculate error
            src_transformed = T_icp[:2, :2] @ src + T_icp[:2, 2, np.newaxis]
            new_distances, _ = nn.kneighbors(src_transformed.T)
            mean_error = np.mean(new_distances)

            # Check for convergence
            if abs(prev_error - mean_error) < self.icp_tolerance:
                 rospy.loginfo(f"[{rospy.get_name()}] ICP converged at iter {i+1} with error {mean_error:.4f}")
                 # Convert 3x3 homogeneous 2D transform to 4x4 homogeneous 3D (z=0, no roll/pitch)
                 T_final_4x4 = np.identity(4)
                 T_final_4x4[0:2, 0:2] = T_icp[0:2, 0:2] # Copy R
                 T_final_4x4[0:2, 3] = T_icp[0:2, 2]   # Copy t (into x, y columns)
                 return T_final_4x4, True # Success

            prev_error = mean_error

        rospy.logwarn(f"[{rospy.get_name()}] ICP failed: Max iterations reached without convergence.")
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
            norm_x = -math.sin(line_angle_rel)
            norm_y = math.cos(line_angle_rel)
            closest_pt_x_rob = line_dist_rel * norm_x
            closest_pt_y_rob = line_dist_rel * norm_y
            line_dir_x = math.cos(line_angle_rel)
            line_dir_y = math.sin(line_angle_rel)
            half_len = self.icp_line_segment_length / 2.0
            start_x_rob = closest_pt_x_rob - half_len * line_dir_x
            start_y_rob = closest_pt_y_rob - half_len * line_dir_y
            end_x_rob = closest_pt_x_rob + half_len * line_dir_x
            end_y_rob = closest_pt_y_rob + half_len * line_dir_y

            # Sample points along the segment in ROBOT frame
            num_samples = int(math.ceil(self.icp_line_segment_length / self.icp_line_sampling_dist)) + 1
            if num_samples < 2: num_samples = 2
            x_samples_rob = np.linspace(start_x_rob, end_x_rob, num_samples)
            y_samples_rob = np.linspace(start_y_rob, end_y_rob, num_samples)
            points_robot = np.vstack((x_samples_rob, y_samples_rob)) # Shape (2, num_samples)

            # Transform points to MAP frame
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

        # Get the corresponding odometry pose for that keyframe (in odom frame)
        if latest_kf_id not in self.keyframe_data:
            rospy.logwarn_throttle(5.0, f"[{rospy.get_name()}] Cannot calculate TF: Keyframe data missing for {latest_kf_id}.")
            return
        odom_pose_latest_kf_ros = self.keyframe_data[latest_kf_id]['odom_pose']
        pos = odom_pose_latest_kf_ros.position
        q = odom_pose_latest_kf_ros.orientation
        _, _, yaw = tf_trans.euler_from_quaternion([q.x, q.y, q.z, q.w])
        pose_odom_latest_kf = Pose2(pos.x, pos.y, yaw) # Pose of latest KF in odom frame

        # Calculate the transform: T_map_odom = T_map_kf * T_kf_odom
        # T_kf_odom = T_odom_kf.inverse()
        pose_kf_odom = pose_odom_latest_kf.inverse()
        map_to_odom_transform = pose_map_latest_kf.compose(pose_kf_odom)

        self.current_map_to_odom = map_to_odom_transform
        rospy.logdebug(f"[{rospy.get_name()}] Updated map->odom TF: x={map_to_odom_transform.x():.3f}, y={map_to_odom_transform.y():.3f}, th={map_to_odom_transform.theta():.3f}")


    def optimize_and_publish_tf(self, event=None):
        """ Periodically called to run optimization (if needed) and publish TF. """
        # Optimization is now done incrementally in odom_callback after adding factors.
        # This timer is mainly for publishing the TF consistently.

        if self.current_map_to_odom is not None:
            t = self.current_map_to_odom.translation()
            r = self.current_map_to_odom.rotation().theta()
            q_tf = tf_trans.quaternion_from_euler(0, 0, r)

            try:
                self.tf_broadcaster.sendTransform(
                    (t.x(), t.y(), 0.0), # translation
                    q_tf,               # rotation
                    rospy.Time.now(),   # timestamp
                    self.odom_frame_id, # child frame
                    self.map_frame_id   # parent frame
                )
            except Exception as e:
                 rospy.logerr_throttle(5.0, f"[{rospy.get_name()}] Error broadcasting TF: {e}")


if __name__ == '__main__':
    try:
        node = PoseGraphOptimizerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Pose graph optimizer node shut down.")
    except Exception as main_e:
        rospy.logfatal(f"Unhandled exception in optimizer main: {main_e}")


