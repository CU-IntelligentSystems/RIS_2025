#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Description: Detects white and yellow line segments, calculates their
#              parameters relative to the robot, and publishes them as
#              LaneLineArray messages. Also publishes debug images.

import rospy
import cv2
import numpy as np
import math
import tf.transformations as tf_trans

# ROS Msgs
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header

try:
    from slam_loopclosure.msg import LaneLine, LaneLineArray
except ImportError:
    rospy.logfatal("Could not import LaneLine messages. "
                   "Make sure the package is built and sourced, ")
    exit()

# ROS Tools
from cv_bridge import CvBridge, CvBridgeError

class LineParameterDetector:
    def __init__(self):
        node_name = 'line_parameter_detector' # Renamed node
        rospy.init_node(node_name, anonymous=True)

        # --- Parameters ---
        # HSV Thresholds
        try:
            self.hsv_lower_white = np.array(rospy.get_param("~hsv_lower_white", [0, 0, 150]), dtype=np.uint8)
            self.hsv_upper_white = np.array(rospy.get_param("~hsv_upper_white", [180, 80, 255]), dtype=np.uint8)
            self.hsv_lower_yellow = np.array(rospy.get_param("~hsv_lower_yellow", [20, 100, 100]), dtype=np.uint8)
            self.hsv_upper_yellow = np.array(rospy.get_param("~hsv_upper_yellow", [40, 255, 255]), dtype=np.uint8)
        except Exception as e:
             rospy.logfatal(f"[{node_name}] Error loading HSV parameters: {e}. Ensure they are lists of 3 numbers.")
             rospy.signal_shutdown("Invalid HSV parameters.")
             return

        # Processing
        self.blur_ksize = rospy.get_param("~blur_ksize", 5)
        self.canny_low_thresh = rospy.get_param("~canny_low_thresh", 50)
        self.canny_high_thresh = rospy.get_param("~canny_high_thresh", 150)
        # Hough
        self.hough_rho = rospy.get_param("~hough_rho", 1)
        self.hough_theta = rospy.get_param("~hough_theta", np.pi/180)
        self.hough_threshold = rospy.get_param("~hough_threshold", 20)
        self.hough_min_line_len = rospy.get_param("~hough_min_line_len", 20)
        self.hough_max_line_gap = rospy.get_param("~hough_max_line_gap", 5)
        # Line Filtering
        self.min_line_slope_abs = rospy.get_param("~min_line_slope_abs", 0.3) # Slope threshold in image frame

        # Camera Calibration (Essential for accurate robot frame conversion)
        # These are EXAMPLE values, replace with your Duckiebot's calibration!
        # You can get these using the camera_calibration ROS package
        try:
            self.camera_matrix = np.array(rospy.get_param("~camera_matrix/data",
                                           [305.5716,    0.    ,  320.5],
                                           [   0.    ,  305.5716,  240.5],
                                           [   0.    ,    0.    ,    1.    ])).reshape((3,3))
            self.dist_coeffs = np.array(rospy.get_param("~distortion_coefficients/data",
                                        [-0.28340811, 0.07334586, 0.0009354 , -0.00011173, 0.        ])).flatten()
        except Exception as e:
             rospy.logwarn(f"[{node_name}] Error loading camera calibration: {e}. Using default values. Robot frame conversion might be inaccurate.")
             # Default values if params fail (less accurate)
             self.camera_matrix = np.array([[300.0, 0.0, 320.0], [0.0, 300.0, 240.0], [0.0, 0.0, 1.0]])
             self.dist_coeffs = np.zeros(5)

        # Homography (Ground Projection) - Assuming flat ground
        # Get this from Duckietown calibration or estimate it. EXAMPLE values.
        # Represents the transform from image pixels to ground plane coords (in meters) relative to camera
        try:
            self.homography_matrix = np.array(rospy.get_param("~homography_matrix",
                                              [[-0.0018,-0.0088, 0.3182],
                                               [-0.0001,-0.0181, 0.1219],
                                               [-0.0000,-0.0436, 1.0000]])).reshape((3,3))
            self.inv_homography = np.linalg.inv(self.homography_matrix) # Used for drawing
        except Exception as e:
            rospy.logwarn(f"[{node_name}] Error loading homography matrix: {e}. Ground projection will be inaccurate.")
            self.homography_matrix = np.eye(3) # Identity placeholder
            self.inv_homography = np.eye(3)

        # Camera Offset from Base Link (Robot Center) - EXAMPLE values
        # X forward, Y left, Z up
        self.cam_offset_x = rospy.get_param("~camera_offset_x", 0.05) # meters
        self.cam_offset_y = rospy.get_param("~camera_offset_y", 0.0)  # meters
        self.cam_offset_z = rospy.get_param("~camera_offset_z", 0.1)  # meters (height)

        rospy.loginfo(f"[{node_name}] Parameters Initialized.")
        rospy.loginfo(f"[{node_name}] Ensure camera_matrix, distortion_coefficients, and homography_matrix parameters are correctly set for accurate results.")


        # --- ROS Comms ---
        self.bridge = CvBridge()
        # Publishers
        self.line_pub = rospy.Publisher("~lines", LaneLineArray, queue_size=1)
        # Debug Image Publishers
        self.pub_white_mask = rospy.Publisher("~image/white_mask", Image, queue_size=2)
        self.pub_yellow_mask = rospy.Publisher("~image/yellow_mask", Image, queue_size=2)
        self.pub_line_overlay = rospy.Publisher("~image/line_overlay", Image, queue_size=2)
        self.pub_undistorted = rospy.Publisher("~image/undistorted", Image, queue_size=2) # For debugging calibration

        # --- Subscriber ---
        image_topic = "/ente/camera_node/image/compressed"
        self.image_sub = rospy.Subscriber(image_topic, CompressedImage,
                                          self.image_callback, queue_size=1, buff_size=2**24)

        rospy.loginfo(f"[{node_name}] Initialization complete. Listening to {image_topic}")

    def transform_pixel_to_robot_frame(self, px, py):
        """ Transforms a single pixel coordinate to the robot's base_link frame (x-forward, y-left). """

        # For simplicity here, assume image is already undistorted in image_callback
        px_undistorted, py_undistorted = px, py

        # 2. Apply Homography (Image pixel -> Ground plane relative to camera)
        pixel_coords = np.array([[px_undistorted, py_undistorted, 1.0]]).T
        ground_coords_cam = self.homography_matrix @ pixel_coords
        if abs(ground_coords_cam[2, 0]) < 1e-6: # Avoid division by zero
            return None
        # Normalize
        ground_x_cam = ground_coords_cam[0, 0] / ground_coords_cam[2, 0]
        ground_y_cam = ground_coords_cam[1, 0] / ground_coords_cam[2, 0]

        # 3. Apply Camera Offset (Ground relative to camera -> Ground relative to robot base_link)
        # Assuming camera x-axis aligns with robot x-axis
        robot_x = ground_x_cam + self.cam_offset_x
        robot_y = ground_y_cam + self.cam_offset_y
        # We are projecting onto the ground plane (z=0 relative to robot base)

        return robot_x, robot_y

    def get_line_params_robot_frame(self, line_segment, img_shape):
        """
        Calculates line parameters (angle, distance) in the robot frame.
        line_segment: [x1, y1, x2, y2] in image pixel coordinates.
        img_shape: (height, width) of the image the segment came from.
        Returns: (angle_rad, distance_m) or None if transformation fails.
        """
        x1_px, y1_px, x2_px, y2_px = line_segment

        # Transform endpoints to robot frame
        pt1_robot = self.transform_pixel_to_robot_frame(x1_px, y1_px)
        pt2_robot = self.transform_pixel_to_robot_frame(x2_px, y2_px)

        if pt1_robot is None or pt2_robot is None:
            rospy.logwarn_throttle(5.0, f"[{rospy.get_name()}] Failed to transform line endpoints to robot frame.")
            return None, None

        x1_rob, y1_rob = pt1_robot
        x2_rob, y2_rob = pt2_robot

        # Calculate line parameters in robot frame (Ax + By + C = 0 form)
        # Handle vertical lines carefully
        if abs(x2_rob - x1_rob) < 1e-6: # Vertical line in robot frame
            angle_rad = np.pi / 2.0 if y2_rob > y1_rob else -np.pi / 2.0
            distance_m = abs(x1_rob) # Distance is the x-coordinate
            # Need to determine sign based on which side of y-axis
            if x1_rob < 0: # Line is to the right (negative y in standard math frame)
                 angle_rad += np.pi # Adjust angle if needed based on convention
                 # Or adjust distance sign based on convention
                 # distance_m = -distance_m # If distance represents signed offset
        else:
            # Standard line equation calculations
            dx = x2_rob - x1_rob
            dy = y2_rob - y1_rob
            line_angle_global = math.atan2(dy, dx) # Angle relative to robot's x-axis

            # Calculate angle of the normal vector (perpendicular to the line)
            # Normal vector: (dy, -dx)
            normal_angle = math.atan2(-dx, dy) # Angle of the normal w.r.t robot's positive y-axis

            # Angle convention: Often, the line's angle is defined relative to the robot's x-axis.
            # Ensure angle is within [-pi, pi]
            angle_rad = line_angle_global
            while angle_rad > np.pi: angle_rad -= 2 * np.pi
            while angle_rad <= -np.pi: angle_rad += 2 * np.pi

            # Calculate perpendicular distance from origin (robot center) to the line
            # Using formula: |Ax0 + By0 + C| / sqrt(A^2 + B^2) where (x0, y0) = (0, 0)
            # A = y1_rob - y2_rob
            # B = x2_rob - x1_rob
            # C = x1_rob * y2_rob - x2_rob * y1_rob
            # distance = abs(C) / math.sqrt(A**2 + B**2)
            # Simpler way: project origin onto line normal
            # Vector from line point 1 to origin: (-x1_rob, -y1_rob)
            # Normal vector (normalized): (cos(normal_angle), sin(normal_angle)) -> This seems wrong. Normal is (dy, -dx)
            norm_vec = np.array([dy, -dx])
            norm_vec_mag = np.linalg.norm(norm_vec)
            if norm_vec_mag < 1e-6: return None, None # Should not happen if dx != 0
            unit_norm_vec = norm_vec / norm_vec_mag

            # Vector from origin to point 1 on line
            vec_orig_to_p1 = np.array([x1_rob, y1_rob])

            # Distance is the projection of vec_orig_to_p1 onto the unit normal vector
            distance_m = abs(np.dot(vec_orig_to_p1, unit_norm_vec))

            # Determine the sign of the distance (optional, depends on convention)
            # If C has the same sign as B, the origin is on one side, otherwise the other.
            # Or check if origin projection lies on the positive/negative normal direction.


        # --- Parameter Filtering (Optional but Recommended) ---
        # Add checks here based on expected line angles/distances in robot frame
        # e.g., if abs(angle_rad) > np.deg2rad(80): return None, None # Filter out lines too parallel to robot x-axis
        # e.g., if distance_m > 0.5: return None, None # Filter lines too far away

        return angle_rad, distance_m


    def image_callback(self, compressed_image_msg):
        """ Detects lines, converts to robot frame params, publishes LaneLineArray. """
        node_name = rospy.get_name()
        current_stamp = compressed_image_msg.header.stamp # Use image stamp

        try:
            np_arr = np.frombuffer(compressed_image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None: raise ValueError("cv_image is None")
        except Exception as e:
            rospy.logerr(f"[{node_name}] Image Decode Error: {e}")
            return

        # --- Undistort Image (Apply calibration) ---
        try:
            h, w = cv_image.shape[:2]
            # Optimize undistortion map calculation (do only once or if calibration changes)
            # For simplicity, recalculating each time here.
            new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h))
            mapx, mapy = cv2.initUndistortRectifyMap(self.camera_matrix, self.dist_coeffs, None, new_camera_mtx, (w, h), 5)
            undistorted_img = cv2.remap(cv_image, mapx, mapy, cv2.INTER_LINEAR)
            # Crop the image based on ROI if needed (optional)
            # x, y, w_roi, h_roi = roi
            # undistorted_img = undistorted_img[y:y+h_roi, x:x+w_roi]
            # h, w = undistorted_img.shape[:2] # Update shape if cropped
        except Exception as e:
            rospy.logerr(f"[{node_name}] Image Undistortion Error: {e}")
            undistorted_img = cv_image # Use original if undistortion fails

        # Publish undistorted image for debugging calibration
        try:
            debug_header = Header(stamp=current_stamp, frame_id=compressed_image_msg.header.frame_id)
            self.pub_undistorted.publish(self.bridge.cv2_to_imgmsg(undistorted_img, encoding="bgr8", header=debug_header))
        except CvBridgeError as e:
             rospy.logerr(f"[{node_name}] CV Bridge Error (Undistorted): {e}")


        # --- Image Processing (on undistorted image) ---
        height, width, _ = undistorted_img.shape
        # Process the relevant part of the image (e.g., bottom half)
        img_roi = undistorted_img[height // 2 : height, :]
        crop_h, crop_w = img_roi.shape[:2]

        # Create Masks
        hsv_image = cv2.cvtColor(img_roi, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(hsv_image, self.hsv_lower_white, self.hsv_upper_white)
        yellow_mask = cv2.inRange(hsv_image, self.hsv_lower_yellow, self.hsv_upper_yellow)

        # Edge Detection
        gray = cv2.cvtColor(img_roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (self.blur_ksize, self.blur_ksize), 0)
        edges = cv2.Canny(blurred, self.canny_low_thresh, self.canny_high_thresh)

        # Filter Edges by Color
        white_edges = cv2.bitwise_and(edges, edges, mask=white_mask)
        yellow_edges = cv2.bitwise_and(edges, edges, mask=yellow_mask)

        # --- Hough Line Segment Detection ---
        white_lines = cv2.HoughLinesP(white_edges, self.hough_rho, self.hough_theta,
                                      self.hough_threshold, np.array([]),
                                      minLineLength=self.hough_min_line_len,
                                      maxLineGap=self.hough_max_line_gap)
        yellow_lines = cv2.HoughLinesP(yellow_edges, self.hough_rho, self.hough_theta,
                                       self.hough_threshold, np.array([]),
                                       minLineLength=self.hough_min_line_len,
                                       maxLineGap=self.hough_max_line_gap)

        # --- Convert Lines to Robot Frame & Publish ---
        lane_lines_msg = LaneLineArray()
        lane_lines_msg.header.stamp = current_stamp
        # Assuming camera frame is aligned with robot frame for simplicity in header
        # Ideally, use tf to know the exact camera frame name relative to base_link
        lane_lines_msg.header.frame_id = "ente/base" # Or your robot's base frame

        detected_lines_for_drawing = [] # Store lines for debug overlay

        # Process White Lines
        if white_lines is not None:
            for line_seg_px in white_lines:
                x1_px, y1_px, x2_px, y2_px = line_seg_px[0]
                # Adjust pixel coords to be relative to the *full* undistorted image
                y1_full = y1_px + height // 2
                y2_full = y2_px + height // 2
                angle_rad, dist_m = self.get_line_params_robot_frame([x1_px, y1_full, x2_px, y2_full], undistorted_img.shape)

                if angle_rad is not None and dist_m is not None:
                    lane_line = LaneLine()
                    lane_line.color = LaneLine.COLOR_WHITE
                    lane_line.angle = angle_rad
                    lane_line.distance = dist_m
                    lane_lines_msg.lines.append(lane_line)
                    # Store original pixel coords for drawing
                    detected_lines_for_drawing.append({'color': 'white', 'pixels': (x1_px, y1_px, x2_px, y2_px)})


        # Process Yellow Lines
        if yellow_lines is not None:
            for line_seg_px in yellow_lines:
                x1_px, y1_px, x2_px, y2_px = line_seg_px[0]
                # Adjust pixel coords to be relative to the *full* undistorted image
                y1_full = y1_px + height // 2
                y2_full = y2_px + height // 2
                angle_rad, dist_m = self.get_line_params_robot_frame([x1_px, y1_full, x2_px, y2_full], undistorted_img.shape)

                if angle_rad is not None and dist_m is not None:
                    lane_line = LaneLine()
                    lane_line.color = LaneLine.COLOR_YELLOW
                    lane_line.angle = angle_rad
                    lane_line.distance = dist_m
                    lane_lines_msg.lines.append(lane_line)
                    # Store original pixel coords for drawing
                    detected_lines_for_drawing.append({'color': 'yellow', 'pixels': (x1_px, y1_px, x2_px, y2_px)})

        # Publish the array of detected lines
        self.line_pub.publish(lane_lines_msg)

        # --- Prepare and Publish Debug Images ---
        line_overlay_img = img_roi.copy() # Draw on the ROI used for detection
        for line_info in detected_lines_for_drawing:
            x1, y1, x2, y2 = line_info['pixels']
            color = (200, 200, 200) if line_info['color'] == 'white' else (0, 200, 200)
            cv2.line(line_overlay_img, (x1, y1), (x2, y2), color, 2)

        # Combine overlay with original ROI image
        combined_img = cv2.addWeighted(img_roi, 0.7, line_overlay_img, 0.3, 0.0)

        try:
            # Use the input image header for debug images
            debug_header = Header(stamp=current_stamp, frame_id=compressed_image_msg.header.frame_id)
            self.pub_white_mask.publish(self.bridge.cv2_to_imgmsg(white_mask, encoding="mono8", header=debug_header))
            self.pub_yellow_mask.publish(self.bridge.cv2_to_imgmsg(yellow_mask, encoding="mono8", header=debug_header))
            # Publish combined overlay relative to the ROI
            self.pub_line_overlay.publish(self.bridge.cv2_to_imgmsg(combined_img, encoding="bgr8", header=debug_header))
        except CvBridgeError as e:
            rospy.logerr(f"[{node_name}] Error publishing debug images: {e}")
        except Exception as e:
             rospy.logerr(f"[{node_name}] Unknown error publishing debug images: {e}")


if __name__ == '__main__':
    try:
        detector = LineParameterDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupt received.")
    except Exception as main_e:
        rospy.logfatal(f"Unhandled exception: {main_e}")


