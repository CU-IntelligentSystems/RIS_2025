#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Description: Detects white line segments using masked Canny+Hough,
#              Averages segments for left/right lines, projects representative points
#              onto the ground plane relative to the robot's base frame,
#              logs coordinates, publishes PointStamped L/R points, and debug images.
#              Includes TF wait logic.

import rospy
import cv2
import numpy as np
import math

# ROS Msgs
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
from geometry_msgs.msg import Point, PointStamped, TransformStamped
from std_msgs.msg import Header

# ROS Tools
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import tf_conversions # Needed for quaternion math
from image_geometry import PinholeCameraModel # Needed for projection
import message_filters # Needed for synchronizing topics

class LineDetectorProjectorAveraged:
    def __init__(self):
       
        node_name = 'line_detector_projector_avg'
        rospy.init_node(node_name, anonymous=True)

        # --- Parameters ---
        try:
            # Correctly load HSV parameters
            hsv_lower_param = rospy.get_param("~hsv_lower_white", [0, 0, 180])
            hsv_upper_param = rospy.get_param("~hsv_upper_white", [180, 50, 255])
            self.hsv_lower_white = np.array([int(v) for v in hsv_lower_param], dtype=np.uint8)
            self.hsv_upper_white = np.array([int(v) for v in hsv_upper_param], dtype=np.uint8)

            # Load other parameters
            self.roi_bottom_width_frac = rospy.get_param("~roi_bottom_width_frac", 0.9)
            self.roi_top_width_frac = rospy.get_param("~roi_top_width_frac", 0.1)
            self.roi_height_frac = rospy.get_param("~roi_height_frac", 0.4)
            self.blur_ksize = rospy.get_param("~blur_ksize", 5)
            self.canny_low_thresh = rospy.get_param("~canny_low_thresh", 50)
            self.canny_high_thresh = rospy.get_param("~canny_high_thresh", 150)
            self.hough_rho = rospy.get_param("~hough_rho", 1)
            self.hough_theta = rospy.get_param("~hough_theta", np.pi/180)
            self.hough_threshold = rospy.get_param("~hough_threshold", 20)
            self.hough_min_line_len = rospy.get_param("~hough_min_line_len", 20)
            self.hough_max_line_gap = rospy.get_param("~hough_max_line_gap", 5)
            self.min_line_slope_abs = rospy.get_param("~min_line_slope_abs", 0.3)
            self.target_frame = rospy.get_param("~target_frame", "ente/base")

            rospy.loginfo(f"[{node_name}]   Parameters Initialized:")
            rospy.loginfo(f"[{node_name}]   HSV Lower (uint8): {self.hsv_lower_white}")
            rospy.loginfo(f"[{node_name}]   HSV Upper (uint8): {self.hsv_upper_white}")
            rospy.loginfo(f"[{node_name}]   Target Frame: {self.target_frame}")

        except ValueError as e:
            rospy.logfatal(f"[{node_name}] Invalid parameter format. Ensure lists contain only numbers. Error: {e}")
            rospy.signal_shutdown("Invalid parameters.")
            return
        except Exception as e:
             rospy.logfatal(f"[{node_name}] Error loading parameters: {e}")
             rospy.signal_shutdown("Parameter loading error.")
             return

        # --- ROS Comms Setup ---
        self.bridge = CvBridge()
        self.cam_model = PinholeCameraModel()
        self.camera_info_received = False
        self.camera_optical_frame = None

        # TF Listener
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Publishers (Separate Left/Right PointStamped)
        self.left_point_pub = rospy.Publisher("/left_line_point_world", PointStamped, queue_size=1)
        self.right_point_pub = rospy.Publisher("/right_line_point_world", PointStamped, queue_size=1)
        # Debug Image Publishers
        self.pub_white_mask = rospy.Publisher("~image/white_mask", Image, queue_size=2)
        self.pub_masked_edges = rospy.Publisher("~image/masked_edges", Image, queue_size=2)
        self.pub_line_overlay = rospy.Publisher("~image/line_overlay", Image, queue_size=2)

        # --- Wait for TF before subscribing ---
        if not self.wait_for_tf_ready():
             rospy.signal_shutdown("Failed to initialize TF transforms.")
             return

        # --- Subscribers (Synchronized) ---
        cam_info_topic = "/ente/camera_node/camera_info"
        image_topic = "/ente/camera_node/image/compressed"
        try:
            cam_info_sub = message_filters.Subscriber(cam_info_topic, CameraInfo)
            image_sub = message_filters.Subscriber(image_topic, CompressedImage)
            # Use ApproximateTimeSynchronizer if timestamps might not be exact
            self.ts = message_filters.ApproximateTimeSynchronizer(
                [image_sub, cam_info_sub], queue_size=10, slop=0.2) # Adjust slop as needed
            self.ts.registerCallback(self.synchronized_callback)
            rospy.loginfo(f"[{node_name}] Subscribed to {image_topic} and {cam_info_topic}")
        except Exception as e:
            rospy.logfatal(f"[{node_name}] Failed to subscribe to topics: {e}")
            rospy.signal_shutdown("Failed to subscribe to topics.")
            return

        rospy.loginfo(f"[{node_name}] Initialization complete. Listening...")


    def wait_for_tf_ready(self):
        """ Waits until CameraInfo is received and the required TF transform is available. """
        node_name = rospy.get_name()
        cam_info_topic = "/ente/camera_node/camera_info"
        try:
            rospy.loginfo(f"[{node_name}] Waiting up to 15s for CameraInfo on {cam_info_topic}...")
            # Make sure CameraInfo is being published
            cam_info_msg = rospy.wait_for_message(cam_info_topic, CameraInfo, timeout=15.0)
            self.camera_optical_frame = cam_info_msg.header.frame_id.lstrip('/')
            self.cam_model.fromCameraInfo(cam_info_msg) # Initialize camera model
            self.camera_info_received = True
            rospy.loginfo(f"[{node_name}] Got CameraInfo for frame: '{self.camera_optical_frame}'.")
        except rospy.ROSException as e:
            rospy.logerr(f"Timeout waiting for CameraInfo on {cam_info_topic}. Is the camera node running? Error: {e}")
            return False

        # Now wait for the transform using the determined frames
        target_tf = self.target_frame
        source_tf = self.camera_optical_frame
        rospy.loginfo(f"[{node_name}] Waiting up to 30s for transform {target_tf} -> {source_tf}...")
        wait_rate = rospy.Rate(1.0)
        start_time = rospy.Time.now()
        timeout_duration = rospy.Duration(30.0) # How long to wait overall

        while not rospy.is_shutdown() and (rospy.Time.now() - start_time) < timeout_duration:
            try:
                # Check if transform is available now, short timeout for check itself
                if self.tf_buffer.can_transform(target_tf, source_tf, rospy.Time(0), timeout=rospy.Duration(0.5)):
                    rospy.loginfo(f"Transform {target_tf} -> {source_tf} is AVAILABLE!")
                    return True # Success
                else:
                    rospy.logwarn_throttle(5.0,f"Still waiting for transform {target_tf} -> {source_tf}...")
            except Exception as e: # Catch potential TF exceptions during check
                rospy.logwarn_throttle(5.0, f"Still waiting for transform {target_tf} -> {source_tf}: {e}")

            wait_rate.sleep()

        rospy.logerr(f"Timeout waiting for transform {target_tf} -> {source_tf}. Check TF publishers (EKF, robot_state_publisher/override) and frame names.")
        return False # Indicate failure


    def get_line_points(self, y, line_params):
        """ Calculates x coordinate on the fitted line for a given y coordinate """
        # line_params is [vx, vy, x0, y0] from cv2.fitLine
        vx, vy, x0, y0 = line_params[0], line_params[1], line_params[2], line_params[3]
        if abs(vy) < 1e-6: return None # Avoid division by zero for horizontal lines
        # Calculate x using the parametric line equation: x = x0 + (vx/vy) * (y - y0)
        x = ((y - y0) * vx / vy) + x0
        return int(round(x)) # Return integer pixel coordinate


    def average_lines(self, lines, img_shape):
        """ Classify, filter, and average line segments using cv2.fitLine """
        node_name = rospy.get_name()
        left_points = []
        right_points = []
        height, width = img_shape[:2] # Cropped image dimensions
        if lines is None: return None, None

        classified_left = 0; classified_right = 0; filtered_slope = 0; filtered_hv = 0

        for i, line in enumerate(lines):
            x1, y1, x2, y2 = line[0]
            if y1 == y2 or x1 == x2: filtered_hv+=1; continue # Filter horizontal/vertical

            slope = float(x2 - x1) / float(y2 - y1) # dx/dy
            if abs(slope) < self.min_line_slope_abs: filtered_slope+=1; continue # Filter near-vertical dx/dy

            # Classify based on slope sign and x-position in CROPPED image
            img_center_x = width / 2.0
            # Check if BOTH points are on the correct side AND slope is correct
            if slope < 0 and x1 < img_center_x and x2 < img_center_x:
                left_points.extend([(x1, y1), (x2, y2)]); classified_left += 1
            elif slope > 0 and x1 > img_center_x and x2 > img_center_x:
                right_points.extend([(x1, y1), (x2, y2)]); classified_right += 1

        # Log filtering/classification summary (use logdebug to reduce spam)
        rospy.logdebug(f"[{node_name}/avg] Raw:{len(lines)} Filt(HV:{filtered_hv},Slope:{filtered_slope}) Class(L:{classified_left}/R:{classified_right}) Pts(L:{len(left_points)}/R:{len(right_points)})")

        left_line_params = None; right_line_params = None
        # Fit line only if enough points collected (>=2 points which means >=1 segment)
        if len(left_points) >= 2:
            left_fit_points = np.array(left_points, dtype=np.int32).reshape(-1, 1, 2)
            try:
                left_line_params = cv2.fitLine(left_fit_points, cv2.DIST_L2, 0, 0.01, 0.01).flatten()
                # rospy.logdebug(f"[{node_name}/avg] Left fitLine OK: {np.round(left_line_params, 3)}") # Uncomment if needed
            except Exception as e: rospy.logwarn(f"[{node_name}/avg] cv2.fitLine failed L: {e}")
        if len(right_points) >= 2:
            right_fit_points = np.array(right_points, dtype=np.int32).reshape(-1, 1, 2)
            try:
                right_line_params = cv2.fitLine(right_fit_points, cv2.DIST_L2, 0, 0.01, 0.01).flatten()
                # rospy.logdebug(f"[{node_name}/avg] Right fitLine OK: {np.round(right_line_params, 3)}") # Uncomment if needed
            except Exception as e: rospy.logwarn(f"[{node_name}/avg] cv2.fitLine failed R: {e}")

        return left_line_params, right_line_params


    def project_pixel_to_ground(self, pixel_coords, stamp):
        """ Projects a pixel coordinate (full image frame) to ground plane (Z=0) in target frame """
        if not self.camera_info_received: return None
        px, py = pixel_coords
        try: ray_cam = np.array(self.cam_model.projectPixelTo3dRay((px, py)))
        except: rospy.logerr(f"projectPixelTo3dRay failed"); return None
        try:
            # Lookup transform from target frame TO camera frame
            trans_stamped = self.tf_buffer.lookup_transform(
                self.target_frame, self.camera_optical_frame, stamp, rospy.Duration(0.1)) # Use message time
            transform = trans_stamped.transform
            # Camera position in target frame
            C = np.array([transform.translation.x, transform.translation.y, transform.translation.z])
            # Rotation that transforms vectors from camera frame TO target frame
            quat = transform.rotation
            R_cam_to_target = tf_conversions.transformations.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])[:3, :3].T

            # Ray direction vector in target frame
            ray_target = R_cam_to_target @ ray_cam

            # Check if ray points towards ground plane (negative Z component in target frame)
            if ray_target[2] >= -1e-6: # Ray is parallel or pointing up relative to target Z
                rospy.logdebug("Ray not pointing towards ground plane (Z=0).")
                return None

            # Calculate distance t along ray to intersection with Z=0 plane
            t = -C[2] / ray_target[2]

            # Check if intersection is within reasonable physical distance
            if t < 0.05 or t > 10.0: # e.g., 5cm to 10m in front
                rospy.logdebug(f"Intersection distance t={t:.2f} out of range.")
                return None

            # Calculate intersection point P in target frame
            P_target = C + t * ray_target

            # Create PointStamped message
            point_msg = PointStamped()
            point_msg.header.stamp = stamp
            point_msg.header.frame_id = self.target_frame
            point_msg.point.x = P_target[0]
            point_msg.point.y = P_target[1]
            point_msg.point.z = P_target[2] # Should be ~0, retain calculated value
            return point_msg

        except Exception as e:
            # Log TF errors less frequently if they occur in callback
            rospy.logwarn_throttle(10.0, f"TF lookup/projection failed in callback: {e}")
            return None


    def synchronized_callback(self, compressed_image_msg, camera_info_msg):
        """ Main processing callback using averaging and projection """
        node_name = rospy.get_name()
        try:
            self.cam_model.fromCameraInfo(camera_info_msg) # Keep model updated for intrinsics
            current_stamp = camera_info_msg.header.stamp
        except Exception as e: rospy.logerr(f"[{node_name}] Callback CameraInfo Error: {e}"); return
        try:
            np_arr = np.frombuffer(compressed_image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None: raise ValueError("cv_image is None")
        except Exception as e: rospy.logerr(f"[{node_name}] Callback Image Decode Error: {e}"); return

        # --- Image Processing Pipeline ---
        height, width, _ = cv_image.shape; height_half = height // 2
        img_bottom = cv_image[height_half:height, :]
        hsv_image = cv2.cvtColor(img_bottom, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(hsv_image, self.hsv_lower_white, self.hsv_upper_white)
        gray = cv2.cvtColor(img_bottom, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (self.blur_ksize, self.blur_ksize), 0)
        edges = cv2.Canny(blurred, self.canny_low_thresh, self.canny_high_thresh)
        white_edges = cv2.bitwise_and(edges, edges, mask=white_mask)
        roi_mask = np.zeros_like(edges); crop_h, crop_w = img_bottom.shape[:2]
        bottom_left = (int(crop_w*(1-self.roi_bottom_width_frac)/2), crop_h-1); bottom_right = (int(crop_w*(1+self.roi_bottom_width_frac)/2), crop_h-1)
        top_left_y = int(crop_h*(1-self.roi_height_frac)); top_left_x = int(crop_w*(1-self.roi_top_width_frac)/2); top_right_x = int(crop_w*(1+self.roi_top_width_frac)/2)
        roi_vertices = np.array([[bottom_left, (top_left_x, top_left_y), (top_right_x, top_left_y), bottom_right]], dtype=np.int32)
        cv2.fillPoly(roi_mask, roi_vertices, 255)
        masked_edges = cv2.bitwise_and(white_edges, roi_mask)
        lines = cv2.HoughLinesP(masked_edges, self.hough_rho, self.hough_theta, self.hough_threshold, np.array([]), minLineLength=self.hough_min_line_len, maxLineGap=self.hough_max_line_gap)

        # --- Average Lines ---
        left_line_params, right_line_params = self.average_lines(lines, img_bottom.shape)

        # --- Calculate Representative Pixel Points ---
        y_bottom_crop = crop_h - 1
        y_top_crop = int(crop_h * 0.6) # For drawing overlay
        left_x_bottom_crop = None; right_x_bottom_crop = None
        if left_line_params is not None: left_x_bottom_crop = self.get_line_points(y_bottom_crop, left_line_params)
        if right_line_params is not None: right_x_bottom_crop = self.get_line_points(y_bottom_crop, right_line_params)

        # --- Prepare Overlay Image ---
        line_img = np.zeros_like(img_bottom)
        if lines is not None: # Draw raw green segments
            for line in lines: cv2.line(line_img, (line[0][0], line[0][1]), (line[0][2], line[0][3]), (0, 255, 0), 1)
        if left_line_params is not None: # Draw averaged blue line
             x_top = self.get_line_points(y_top_crop, left_line_params)
             if left_x_bottom_crop is not None and x_top is not None: cv2.line(line_img, (left_x_bottom_crop, y_bottom_crop), (x_top, y_top_crop), (255, 0, 0), 3)
        if right_line_params is not None: # Draw averaged red line
             x_top = self.get_line_points(y_top_crop, right_line_params)
             if right_x_bottom_crop is not None and x_top is not None: cv2.line(line_img, (right_x_bottom_crop, y_bottom_crop), (x_top, y_top_crop), (0, 0, 255), 3)
        cv2.polylines(line_img, [roi_vertices], isClosed=True, color=(255,255,0), thickness=1) # ROI
        combined_img = cv2.addWeighted(img_bottom, 0.8, line_img, 1.0, 0.0)

        # --- Project Averaged Points and Publish ---
        log_output = f"[{node_name}] World Pts ({self.target_frame}): "
        left_x_world, left_y_world = None, None
        right_x_world, right_y_world = None, None

        if left_x_bottom_crop is not None:
            left_px_full = left_x_bottom_crop; left_py_full = y_bottom_crop + height_half
            left_point_world_msg = self.project_pixel_to_ground((left_px_full, left_py_full), current_stamp)
            if left_point_world_msg:
                self.left_point_pub.publish(left_point_world_msg)
                left_x_world = left_point_world_msg.point.x; left_y_world = left_point_world_msg.point.y
                log_output += f"L(X:{left_x_world:.3f},Y:{left_y_world:.3f}) "
            else: log_output += "L=ProjFail "
        else: log_output += "L=None "

        if right_x_bottom_crop is not None:
            right_px_full = right_x_bottom_crop; right_py_full = y_bottom_crop + height_half
            right_point_world_msg = self.project_pixel_to_ground((right_px_full, right_py_full), current_stamp)
            if right_point_world_msg:
                self.right_point_pub.publish(right_point_world_msg)
                right_x_world = right_point_world_msg.point.x; right_y_world = right_point_world_msg.point.y
                log_output += f"R(X:{right_x_world:.3f},Y:{right_y_world:.3f})"
            else: log_output += "R=ProjFail"
        else: log_output += "R=None"

        rospy.loginfo(log_output) # Log results for this frame

        # --- Publish Debug Images ---
        try:
            debug_header = Header(stamp=current_stamp, frame_id=self.camera_optical_frame)
            self.pub_white_mask.publish(self.bridge.cv2_to_imgmsg(white_mask, encoding="mono8", header=debug_header))
            self.pub_masked_edges.publish(self.bridge.cv2_to_imgmsg(masked_edges, encoding="mono8", header=debug_header))
            self.pub_line_overlay.publish(self.bridge.cv2_to_imgmsg(combined_img, encoding="bgr8", header=debug_header))
        except Exception as e: rospy.logerr(f"Error publishing debug images: {e}")


if __name__ == '__main__':
    try:
        ldpa = LineDetectorProjectorAveraged()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupt received. Shutting down.")
    except Exception as main_e:
        # Log any other exceptions during setup or runtime
        rospy.logfatal(f"Unhandled exception in main: {main_e}")
