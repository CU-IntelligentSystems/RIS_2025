#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import math

class SimpleLaneFollower(Node):
    def __init__(self):
        super().__init__('simple_lane_follower')
        
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/duckiebot/camera/image_raw', self.image_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/duckiebot/cmd_vel', 10)
        self.debug_image_pub = self.create_publisher(Image, '/lane_debug', 10)
        self.lane_detected_pub = self.create_publisher(Bool, '/lane_detected', 10)
        
        # Control parameters
        self.straight_speed = 0.5      # Fast speed for straight lines
        self.turn_speed = 0.15         # Slower speed for turns
        self.min_speed = 0.08          # Minimum speed
        
        # Steering parameters
        self.max_angular_speed = 1.8
        self.steering_gain = 2.5
        
        # Yellow line detection parameters
        self.yellow_hsv_low = np.array([10, 40, 80])
        self.yellow_hsv_high = np.array([40, 255, 255])
        
        # State tracking
        self.lane_detected = False
        self.lane_center = None
        self.image_center = 320  # Assume 640px width
        
        # Smoothing for speed control
        self.recent_errors = []
        self.error_history_size = 5
        
        self.get_logger().info("🏁 Simple Lane Follower started!")
        self.get_logger().info(f"   Straight speed: {self.straight_speed}m/s")
        self.get_logger().info(f"   Turn speed: {self.turn_speed}m/s")
        
    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Detect yellow line
            lane_center, lane_detected = self.detect_yellow_line(cv_image)
            
            # Control robot based on detection
            if lane_detected:
                self.follow_lane(lane_center, cv_image.shape[1])
                self.lane_detected = True
                self.lane_center = lane_center
            else:
                self.handle_no_line()
                self.lane_detected = False
            
            # Publish lane detection status
            detected_msg = Bool()
            detected_msg.data = self.lane_detected
            self.lane_detected_pub.publish(detected_msg)
            
            # Create and publish debug image
            debug_img = self.create_debug_image(cv_image, lane_center, lane_detected)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")
            self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")
    
    def detect_yellow_line(self, image):
        """Detect yellow line and return center position"""
        height, width = image.shape[:2]
        
        # Focus on bottom half of image
        roi_start = int(height * 0.5)
        roi = image[roi_start:, :]
        
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Create mask for yellow
        mask = cv2.inRange(hsv, self.yellow_hsv_low, self.yellow_hsv_high)
        
        # Clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Find largest contour (should be the yellow line)
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            if area > 200:  # Minimum area threshold
                # Calculate center of mass
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    return cx, True
        
        return None, False
    
    def follow_lane(self, lane_center, image_width):
        """Control robot to follow the detected lane"""
        cmd = Twist()
        
        # Calculate error (how far off-center we are)
        image_center = image_width // 2
        error = (lane_center - image_center) / image_center  # Normalize to [-1, 1]
        
        # Add to error history for smoothing
        self.recent_errors.append(abs(error))
        if len(self.recent_errors) > self.error_history_size:
            self.recent_errors.pop(0)
        
        # Calculate average recent error (for speed control)
        avg_error = np.mean(self.recent_errors)
        
        # Adaptive speed based on how straight the path is
        if avg_error < 0.1:  # Very straight
            speed = self.straight_speed
        elif avg_error < 0.3:  # Moderate curve
            speed = self.turn_speed
        else:  # Sharp turn
            speed = self.min_speed
        
        # Steering control (proportional controller)
        angular_velocity = -self.steering_gain * error
        
        # Limit angular velocity
        angular_velocity = max(-self.max_angular_speed, 
                             min(self.max_angular_speed, angular_velocity))
        
        # Additional speed reduction for sharp steering
        if abs(angular_velocity) > 0.8:
            speed *= 0.7
        
        # Set velocities
        cmd.linear.x = speed
        cmd.angular.z = angular_velocity
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
        
        # Log status occasionally
        if hasattr(self, '_log_counter'):
            self._log_counter += 1
        else:
            self._log_counter = 0
            
        if self._log_counter % 30 == 0:  # Every ~1 second at 30fps
            self.get_logger().info(
                f"🏁 Following: error={error:.3f}, speed={speed:.2f}, "
                f"steering={angular_velocity:.2f}, avg_error={avg_error:.3f}"
            )
    
    def handle_no_line(self):
        """What to do when no line is detected"""
        cmd = Twist()
        
        # If we recently had a line, keep the last steering direction briefly
        if hasattr(self, '_lost_line_count'):
            self._lost_line_count += 1
        else:
            self._lost_line_count = 0
        
        if self._lost_line_count < 10:  # Brief continuation
            if self.lane_center is not None:
                # Keep turning in the last known direction, but slowly
                last_error = (self.lane_center - self.image_center) / self.image_center
                cmd.linear.x = self.min_speed
                cmd.angular.z = -0.5 * last_error  # Gentle continuation
            else:
                cmd.linear.x = self.min_speed  # Just move forward slowly
        else:
            # Stop if lost for too long
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
            if self._lost_line_count % 30 == 0:
                self.get_logger().warn("⚠️ No yellow line detected - stopping")
        
        self.cmd_vel_pub.publish(cmd)
    
    def create_debug_image(self, image, lane_center, lane_detected):
        """Create debug visualization"""
        debug = image.copy()
        height, width = debug.shape[:2]
        
        # Draw ROI
        roi_start = int(height * 0.5)
        cv2.rectangle(debug, (0, roi_start), (width, height), (255, 255, 255), 2)
        
        # Draw center line
        image_center = width // 2
        cv2.line(debug, (image_center, roi_start), (image_center, height), (0, 0, 255), 2)
        cv2.putText(debug, "CENTER", (image_center - 30, roi_start - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        # Draw detected lane center
        if lane_detected and lane_center is not None:
            cv2.line(debug, (lane_center, roi_start), (lane_center, height), (0, 255, 0), 3)
            cv2.putText(debug, "LANE", (lane_center - 20, roi_start - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            # Calculate and show error
            error = (lane_center - image_center) / image_center
            cv2.putText(debug, f"Error: {error:.3f}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Show status
        status = "DETECTED" if lane_detected else "LOST"
        color = (0, 255, 0) if lane_detected else (0, 0, 255)
        cv2.putText(debug, f"Lane: {status}", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Show current speed setting
        if hasattr(self, 'recent_errors') and self.recent_errors:
            avg_error = np.mean(self.recent_errors)
            if avg_error < 0.1:
                speed_text = f"STRAIGHT ({self.straight_speed}m/s)"
                speed_color = (0, 255, 0)
            elif avg_error < 0.3:
                speed_text = f"TURN ({self.turn_speed}m/s)"
                speed_color = (0, 255, 255)
            else:
                speed_text = f"SHARP ({self.min_speed}m/s)"
                speed_color = (0, 128, 255)
            
            cv2.putText(debug, speed_text, (10, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, speed_color, 2)
        
        return debug

def main(args=None):
    rclpy.init(args=args)
    
    follower = SimpleLaneFollower()
    
    try:
        rclpy.spin(follower)
    except KeyboardInterrupt:
        pass
    finally:
        follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()