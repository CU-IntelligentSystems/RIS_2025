#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Description: Detects white and yellow line segments in the BOTTOM HALF of image
#              using masked Canny+Hough. Averages segments for left/right lines
#              for each color. Publishes the status (White=0, Yellow=1, None=-1)
#              for each side. Publishes debug images.

import rospy
import cv2
import numpy as np
import math

# ROS Msgs
from sensor_msgs.msg import CompressedImage, Image # Only need Image for debug publish
from std_msgs.msg import Int8, Header # Use Int8 for status

# ROS Tools
from cv_bridge import CvBridge, CvBridgeError
# TF/Projection related imports REMOVED

class LineStatusDetectorNoROI:
    def __init__(self):
        node_name = 'line_status_detector_no_roi'
        rospy.init_node(node_name, anonymous=True)

        self.veh = rospy.get_param("~veh","ente")

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
        self.min_line_slope_abs = rospy.get_param("~min_line_slope_abs", 0.3)

        rospy.loginfo(f"[{node_name}] Parameters Initialized.")
        rospy.loginfo(f"[{node_name}] HSV White Low: {self.hsv_lower_white} High: {self.hsv_upper_white}")
        rospy.loginfo(f"[{node_name}] HSV Yellow Low: {self.hsv_lower_yellow} High: {self.hsv_upper_yellow}")


        # --- ROS Comms ---
        self.bridge = CvBridge()

        # Publishers
        self.left_status_pub = rospy.Publisher("~line/left/status", Int8, queue_size=1)
        self.right_status_pub = rospy.Publisher("~line/right/status", Int8, queue_size=1)
        # Debug Image Publishers
        self.pub_white_mask = rospy.Publisher("~image/white_mask", Image, queue_size=2)
        self.pub_yellow_mask = rospy.Publisher("~image/yellow_mask", Image, queue_size=2)
        self.pub_color_filtered_edges = rospy.Publisher("~image/color_filtered_edges", Image, queue_size=2)
        self.pub_line_overlay = rospy.Publisher("~image/line_overlay", Image, queue_size=2)

        # --- Subscriber---
        image_topic = f"/{self.veh}/camera_node/image/compressed" 
        self.image_sub = rospy.Subscriber(image_topic, CompressedImage,
                                          self.image_callback, queue_size=1, buff_size=2**24)

        rospy.loginfo(f"[{node_name}] Initialization complete. Listening to {image_topic}")


    # --- Helper Functions ---
    def get_line_points(self, y, line_params):
        # Still needed for drawing the overlay
        if line_params is None: return None
        vx, vy, x0, y0 = line_params[0], line_params[1], line_params[2], line_params[3]
        if abs(vy) < 1e-6: return None
        x = ((y - y0) * vx / vy) + x0
        return int(round(x))

    def average_lines(self, lines, img_shape):
        # (Same average_lines function as before - classifies L/R based on img_shape width)
        node_name = rospy.get_name(); left_points = []; right_points = []
        height, width = img_shape[:2]; img_center_x = width / 2.0
        if lines is None: return None, None
        classified_left = 0; classified_right = 0; filtered_slope = 0; filtered_hv = 0

        for i, line in enumerate(lines):
            x1, y1, x2, y2 = line[0]
            if y1 == y2 or x1 == x2: filtered_hv+=1; continue
            # Use try-except for slope calculation robustness
            try: slope = float(x2 - x1) / float(y2 - y1)
            except ZeroDivisionError: filtered_hv+=1; continue

            if abs(slope) < self.min_line_slope_abs: filtered_slope+=1; continue

            if slope < 0 and x1 < img_center_x and x2 < img_center_x:
                left_points.extend([(x1, y1), (x2, y2)]); classified_left += 1
            elif slope > 0 and x1 > img_center_x and x2 > img_center_x:
                right_points.extend([(x1, y1), (x2, y2)]); classified_right += 1

        # Log filtering/classification summary (use logdebug to reduce spam)
        rospy.logdebug(f"[{node_name}/avg] Raw:{len(lines)} Filt(HV:{filtered_hv},Slope:{filtered_slope}) Class(L:{classified_left}/R:{classified_right}) Pts(L:{len(left_points)}/R:{len(right_points)})")

        left_line_params = None; right_line_params = None
        if len(left_points) >= 2:
            left_fit_points = np.array(left_points, dtype=np.int32).reshape(-1, 1, 2)
            try: left_line_params = cv2.fitLine(left_fit_points, cv2.DIST_L2, 0, 0.01, 0.01).flatten()
            except Exception as e: rospy.logwarn(f"[{node_name}/avg] cv2.fitLine failed L: {e}")
        if len(right_points) >= 2:
            right_fit_points = np.array(right_points, dtype=np.int32).reshape(-1, 1, 2)
            try: right_line_params = cv2.fitLine(right_fit_points, cv2.DIST_L2, 0, 0.01, 0.01).flatten()
            except Exception as e: rospy.logwarn(f"[{node_name}/avg] cv2.fitLine failed R: {e}")
        return left_line_params, right_line_params


    def image_callback(self, compressed_image_msg):
        """ Detects white/yellow lines in bottom half, classifies L/R, publishes status. """
        node_name = rospy.get_name()
        try:
            np_arr = np.frombuffer(compressed_image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None: raise ValueError("cv_image is None")
            current_stamp = compressed_image_msg.header.stamp # Use image stamp
        except Exception as e: rospy.logerr(f"[{node_name}] Image Decode Error: {e}"); return

        # --- Image Processing ---
        height, width, _ = cv_image.shape; height_half = height // 2
        img_bottom = cv_image[height_half:height, :]
        crop_h, crop_w = img_bottom.shape[:2] # Get dimensions of cropped image

        # Create Masks
        hsv_image = cv2.cvtColor(img_bottom, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(hsv_image, self.hsv_lower_white, self.hsv_upper_white)
        yellow_mask = cv2.inRange(hsv_image, self.hsv_lower_yellow, self.hsv_upper_yellow)

        # Edge Detection
        gray = cv2.cvtColor(img_bottom, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (self.blur_ksize, self.blur_ksize), 0)
        edges = cv2.Canny(blurred, self.canny_low_thresh, self.canny_high_thresh)

        # Filter Edges by Color
        white_edges = cv2.bitwise_and(edges, edges, mask=white_mask)
        yellow_edges = cv2.bitwise_and(edges, edges, mask=yellow_mask)

        # Combine edges for debug view
        all_color_filtered_edges = cv2.bitwise_or(white_edges, yellow_edges)

        # --- Hough Line Segment Detection (Separate for White and Yellow) ---
        # Run on white edges
        white_lines = cv2.HoughLinesP(white_edges, self.hough_rho, self.hough_theta,
                                      self.hough_threshold, np.array([]),
                                      minLineLength=self.hough_min_line_len,
                                      maxLineGap=self.hough_max_line_gap)
        # Run on yellow edges
        yellow_lines = cv2.HoughLinesP(yellow_edges, self.hough_rho, self.hough_theta,
                                       self.hough_threshold, np.array([]),
                                       minLineLength=self.hough_min_line_len,
                                       maxLineGap=self.hough_max_line_gap)

        # --- Average Lines Separately ---
        white_left_params, white_right_params = self.average_lines(white_lines, img_bottom.shape)
        yellow_left_params, yellow_right_params = self.average_lines(yellow_lines, img_bottom.shape)

        # --- Determine Left/Right Status ---
        left_status = -1 # Default: None
        if white_left_params is not None: left_status = 0 # White
        elif yellow_left_params is not None: left_status = 1 # Yellow

        right_status = -1 # Default: None
        if white_right_params is not None: right_status = 0 # White
        elif yellow_right_params is not None: right_status = 1 # Yellow

        # --- Publish Status ---
        self.left_status_pub.publish(Int8(left_status))
        self.right_status_pub.publish(Int8(right_status))
        rospy.loginfo(f"[{node_name}] Status L/R: {left_status}/{right_status}") # Log status

        # --- Prepare and Publish Debug Images ---
        line_img = np.zeros_like(img_bottom)
        y_bottom_crop = crop_h - 1; y_top_crop = int(crop_h * 0.6) # Define Y levels for drawing avg lines

        # Draw raw segments (White=Gray, Yellow=Light Yellow/Cyan)
        if white_lines is not None:
            for line in white_lines: cv2.line(line_img, (line[0][0], line[0][1]), (line[0][2], line[0][3]), (200, 200, 200), 1)
        if yellow_lines is not None:
            for line in yellow_lines: cv2.line(line_img, (line[0][0], line[0][1]), (line[0][2], line[0][3]), (0, 200, 200), 1)

        # Draw averaged lines (W: Blue/Red, Y: Magenta/Cyan)
        if white_left_params is not None:
             x_bottom = self.get_line_points(y_bottom_crop, white_left_params); x_top = self.get_line_points(y_top_crop, white_left_params)
             if x_bottom is not None and x_top is not None: cv2.line(line_img, (x_bottom, y_bottom_crop), (x_top, y_top_crop), (255, 0, 0), 3) # Blue
        if white_right_params is not None:
             x_bottom = self.get_line_points(y_bottom_crop, white_right_params); x_top = self.get_line_points(y_top_crop, white_right_params)
             if x_bottom is not None and x_top is not None: cv2.line(line_img, (x_bottom, y_bottom_crop), (x_top, y_top_crop), (0, 0, 255), 3) # Red
        if yellow_left_params is not None:
             x_bottom = self.get_line_points(y_bottom_crop, yellow_left_params); x_top = self.get_line_points(y_top_crop, yellow_left_params)
             if x_bottom is not None and x_top is not None: cv2.line(line_img, (x_bottom, y_bottom_crop), (x_top, y_top_crop), (255, 0, 255), 3) # Magenta
        if yellow_right_params is not None:
             x_bottom = self.get_line_points(y_bottom_crop, yellow_right_params); x_top = self.get_line_points(y_top_crop, yellow_right_params)
             if x_bottom is not None and x_top is not None: cv2.line(line_img, (x_bottom, y_bottom_crop), (x_top, y_top_crop), (255, 255, 0), 3) # Cyan

        combined_img = cv2.addWeighted(img_bottom, 0.8, line_img, 1.0, 0.0)

        try:
            # Use the input image header for debug images
            debug_header = compressed_image_msg.header
            self.pub_white_mask.publish(self.bridge.cv2_to_imgmsg(white_mask, encoding="mono8", header=debug_header))
            self.pub_yellow_mask.publish(self.bridge.cv2_to_imgmsg(yellow_mask, encoding="mono8", header=debug_header))
            self.pub_color_filtered_edges.publish(self.bridge.cv2_to_imgmsg(all_color_filtered_edges, encoding="mono8", header=debug_header))
            self.pub_line_overlay.publish(self.bridge.cv2_to_imgmsg(combined_img, encoding="bgr8", header=debug_header))
        except Exception as e: rospy.logerr(f"Error publishing debug images: {e}")


if __name__ == '__main__':
    try:
        lsd_no_roi = LineStatusDetectorNoROI()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupt received.")
    except Exception as main_e:
        rospy.logfatal(f"Unhandled exception: {main_e}")