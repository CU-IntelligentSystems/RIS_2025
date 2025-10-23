#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist, PoseStamped, Pose, TransformStamped
from duckietown_msgs.msg import AprilTagDetectionArray, WheelsCmdStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import tf2_ros
import tf2_geometry_msgs
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
from cv_bridge import CvBridge

class IntegratedDuckieController:
    def __init__(self):
        rospy.init_node('integrated_duckie_controller', anonymous=True)
        self.robot_name = rospy.get_param('~veh', 'lucky')
        self.running = True

        # Initializing the CV bridge
        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = rospy.Subscriber(f'/{self.robot_name}/camera_node/image/compressed', 
                                        CompressedImage, 
                                        self.image_callback)
        self.tag_sub = rospy.Subscriber('/apriltag_detections', 
                                       AprilTagDetectionArray, 
                                       self.tag_callback)
        self.pose_sub = rospy.Subscriber('/apriltag_poses', 
                                        PoseStamped, 
                                        self.pose_callback)
        self.odom_sub = rospy.Subscriber(f'/{self.robot_name}/odometry', 
                                        Odometry, 
                                        self.odometry_callback)
        self.message_sub = rospy.Subscriber('/apriltag_messages', 
                                          String, 
                                          self.message_callback)

        # Publishers
        self.wheel_pub = rospy.Publisher(f'/{self.robot_name}/wheels_driver_node/wheels_cmd', 
                                       WheelsCmdStamped, 
                                       queue_size=10)
        self.debug_pub = rospy.Publisher('/debug_lane_image/compressed', 
                                       CompressedImage, 
                                       queue_size=10)
        self.marker_pub = rospy.Publisher('/apriltag_markers', 
                                        MarkerArray, 
                                        queue_size=10)
        self.optimized_pose_pub = rospy.Publisher('/optimized_robot_pose', 
                                                 PoseStamped, 
                                                 queue_size=10)

        # Control Parameters
        self.base_speed = rospy.get_param('~base_speed', 0.35)    # normal speed
        self.max_speed = rospy.get_param('~max_speed', 0.45)      # max speed limit
        self.kp = rospy.get_param('~kp', 0.2)                    # proportional gain (response to error)
        self.ki = rospy.get_param('~ki', 0.01)                   # integral gain (accumulated error)
        self.kd = rospy.get_param('~kd', 0.08)                   # Increased derivative (for better damping)
        self.integral_limit = rospy.get_param('~integral_limit', 0.2)
        self.smoothing_factor = rospy.get_param('~smoothing_factor', 0.4)  # Smoothing factor
        self.corner_speed_factor = rospy.get_param('~corner_speed_factor', 0.6)  # Speed reduction in corners
        
        # Line Detection Parameters for detecting white lines
        self.white_lower = np.array([0, 0, 200])  
        self.white_upper = np.array([180, 50, 255])  
        self.roi_height = rospy.get_param('~roi_height', 2)
        self.line_threshold = rospy.get_param('~line_threshold', 200)

        # State variables for actions
        self.tag_detected = False
        self.tag_action = None
        self.line_detected = False
        self.line_position = 0.0
        self.integral_error = 0.0
        self.last_time = rospy.Time.now()
        self.current_tag_distance = float('inf')
        self.current_tag_id = None
        self.should_stop = False
        self.last_error = 0.0  # For derivative control
        self.smoothed_control = 0.0  # For control output smoothing
        
        # Tag handling parameters
        self.tag_stop_duration = rospy.Duration(2.0)  # Stops duckiebot 2 seconds when tag detected
        self.tag_cooldown_duration = rospy.Duration(10.0)  # Ignores tags for 10 seconds after action
        self.last_tag_time = rospy.Time.now()
        self.tag_action_active = False
        self.tag_cooldown_active = False

        # SLAM variables (which were not used)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tag_poses = {}
        self.robot_poses = []
        self.tag_observations = []
        self.current_pose_key = 0
        self.last_odom_pose = None
        self.optimization_interval = rospy.Duration(1.0)
        self.last_optimization_time = rospy.Time.now()
        self.marker_array = MarkerArray()
        self.marker_id = 0

        rospy.on_shutdown(self.shutdown) # shuts down the node when the program is closed
        rospy.loginfo("Integrated Duckie Controller Started") # logs the start of the node

    def shutdown(self): # shuts down the node when the program is closed
        """Clean shutdown of the node"""
        rospy.loginfo("Shutting down Integrated Duckie Controller...")
        self.running = False
        self.send_wheel_command(0.0, 0.0)

    def detect_line(self, image): # detects the white line using HSV thresholding and returns the line position
        """Detect white line using HSV thresholding and return line position"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # converts the image to HSV color space
        mask = cv2.inRange(hsv, self.white_lower, self.white_upper) # creates a mask for the white line
        
        height, width = mask.shape # gets the height and width of the mask
        roi = mask[height - height//self.roi_height:, :] # gets the region of interest (ROI)
        
        # Creating debug visualization
        debug_image = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)
        
        # Find line center using weighted average
        white_pixels = np.where(roi > self.line_threshold)
        if len(white_pixels[1]) > 0:
            weights = np.exp(-0.1 * (height - white_pixels[0]))
            weighted_x = np.average(white_pixels[1], weights=weights)
            
            self.line_detected = True
            line_position = (weighted_x - width/2) / (width/2)  # Normalize to [-1, 1]
            
            # Drawing debug visualization
            cv2.line(debug_image, (int(weighted_x), 0), 
                    (int(weighted_x), debug_image.shape[0]), (0, 255, 0), 2)
            cv2.putText(debug_image, f"Center: {weighted_x:.1f}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            self.line_detected = False
            line_position = 0.0
            cv2.putText(debug_image, "No line detected", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Publishing debug image
        _, encoded = cv2.imencode('.jpg', debug_image)
        debug_msg = CompressedImage()
        debug_msg.header.stamp = rospy.Time.now()
        debug_msg.format = "jpeg"
        debug_msg.data = encoded.tobytes()
        self.debug_pub.publish(debug_msg)

        return line_position

    def control_robot(self, line_position):
        """
        Control the robot based on line position and tag detection.
        
        Args:
            line_position (float): Normalized position of the line (-1 to 1)
                                 where 0 is center, -1 is far left, 1 is far right
        """
        if not self.running:
            self.send_wheel_command(0.0, 0.0)
            self.integral_error = 0.0
            self.smoothed_control = 0.0
            return

        current_time = rospy.Time.now()

        # Tag detection and actions
        if self.tag_detected and not self.tag_cooldown_active:
            time_since_tag = current_time - self.last_tag_time
            
            # If duckiebot is within the stop duration, stop the robot
            if time_since_tag < self.tag_stop_duration:
                self.tag_action_active = True
                # Different actions for different tags (turning for turns, stopping for others), and printing messages
                if self.tag_action and "slow down" in self.tag_action:
                    self.send_wheel_command(0.0, 0.0)  
                    rospy.loginfo("Slowing down for tag")
                elif self.tag_action and "stop" in self.tag_action:
                    self.send_wheel_command(0.0, 0.0)  
                    rospy.loginfo(f"Stopping for tag action: {self.tag_action}")
                elif self.tag_action and "yield" in self.tag_action:
                    self.send_wheel_command(0.0, 0.0)  
                    rospy.loginfo(f"Stopping for tag action: {self.tag_action}")
                elif self.tag_action and "wait" in self.tag_action:
                    self.send_wheel_command(0.0, 0.0)  
                    rospy.loginfo(f"Stopping for tag action: {self.tag_action}")
                elif self.tag_action and "go right" in self.tag_action:
                    self.send_wheel_command(0.2, -0.2)  
                    rospy.loginfo(f"Stopping for tag action: {self.tag_action}")
                elif self.tag_action and "go left" in self.tag_action:
                    self.send_wheel_command(-0.2, 0.2)  
                    rospy.loginfo(f"Stopping for tag action: {self.tag_action}")
                return
            else:
                # When stop duration is over, activates cooldown and resume normal moving
                self.tag_action_active = False
                self.tag_cooldown_active = True
                self.last_tag_time = current_time  # Reset timer for cooldown
                rospy.loginfo("Tag stop complete, entering cooldown period")

        # Checking if bot is in cooldown period
        if self.tag_cooldown_active:
            time_since_action = current_time - self.last_tag_time
            if time_since_action >= self.tag_cooldown_duration:
                self.tag_cooldown_active = False
                self.tag_detected = False
                self.tag_action = None
                rospy.loginfo("Cooldown period complete, resuming normal operation")

        # Normal line following if no tag action is active
        if self.line_detected:
            # Line following control with improved corner turning
            dt = (current_time - self.last_time).to_sec()
            self.last_time = current_time

            # Rest of the line following code is the same
            derivative = (line_position - self.last_error) / max(dt, 0.01)
            derivative = np.clip(derivative, -1.0, 1.0)
            self.last_error = line_position

            self.integral_error += line_position * dt
            self.integral_error = np.clip(self.integral_error, 
                                        -self.integral_limit, 
                                        self.integral_limit)

            control_output = (self.kp * line_position + 
                            self.ki * self.integral_error +
                            self.kd * derivative)
            
            self.smoothed_control = (self.smoothing_factor * self.smoothed_control + 
                                   (1 - self.smoothing_factor) * control_output)
            
            max_control = 0.3
            self.smoothed_control = np.clip(self.smoothed_control, -max_control, max_control)
            
            turn_severity = (abs(self.smoothed_control) / max_control) ** 2
            corner_reduction = 1.0 - (turn_severity * self.corner_speed_factor)
            
            current_base_speed = self.base_speed * corner_reduction
            min_base_speed = 0.15
            current_base_speed = max(current_base_speed, min_base_speed)
            
            turn_boost = 0.0
            if abs(line_position) > 0.5:
                turn_boost = 0.1 * np.sign(self.smoothed_control)
            
            left_speed = current_base_speed - self.smoothed_control - turn_boost
            right_speed = current_base_speed + self.smoothed_control + turn_boost
            
            left_speed = np.clip(left_speed, -self.max_speed, self.max_speed)
            right_speed = np.clip(right_speed, -self.max_speed, self.max_speed)
            
            self.send_wheel_command(left_speed, right_speed)
        else:
            # Search mode with forward bias
            self.send_wheel_command(0.15, 0.05)
            self.integral_error = 0.0
            self.smoothed_control = 0.0

    def send_wheel_command(self, left_speed, right_speed):
        """Send wheel commands to the robot"""
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.Time.now()
        msg.vel_left = left_speed
        msg.vel_right = right_speed
        self.wheel_pub.publish(msg)
        
        # Printing wheel speeds 
        rospy.loginfo(f"Wheel speeds - Left: {left_speed:.3f}, Right: {right_speed:.3f}")

    def image_callback(self, msg):
        """Process camera image for line following"""
        if not self.running:
            return

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            line_position = self.detect_line(image)
            self.control_robot(line_position)
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def tag_callback(self, msg):
        """Handle AprilTag detections"""
        try:
            # Check if duckiebot in the cooldown period
            if self.tag_cooldown_active:
                time_since_action = rospy.Time.now() - self.last_tag_time
                if time_since_action < self.tag_cooldown_duration:
                    # If still in cooldown, ignores tag detections
                    return
                else:
                    # Cooldown period is over
                    self.tag_cooldown_active = False

            if len(msg.detections) > 0:
                self.tag_detected = True
                self.current_tag_id = msg.detections[0].tag_id
                self.last_tag_time = rospy.Time.now()
                
                # Store only necessary tag information
                self.tag_observations.append((
                    self.current_pose_key,
                    self.current_tag_id,
                    msg.detections[0].tag_family
                ))
                
                rospy.loginfo(f"Detected AprilTag {self.current_tag_id}")
            else:
                # Only reset tag detection if not in an active tag action
                if not self.tag_action_active and not self.tag_cooldown_active:
                    self.tag_detected = False
                    self.current_tag_id = None
                    self.tag_action = None
        except Exception as e:
            rospy.logerr(f"Error in tag_callback: {e}")

    def message_callback(self, msg):
        """Handle AprilTag messages"""
        try:
            self.tag_action = msg.data.lower()
            rospy.loginfo(f"Received tag action: {self.tag_action}")
        except Exception as e:
            rospy.logerr(f"Error in message_callback: {e}")

    def pose_callback(self, msg):
        """Handle pose updates"""
        if msg.header.frame_id in self.tag_poses:
            self.tag_poses[msg.header.frame_id].append(msg.pose)
        else:
            self.tag_poses[msg.header.frame_id] = [msg.pose]

    def odometry_callback(self, msg):
        """Handle odometry updates"""
        current_odom_pose = msg.pose.pose
        if self.last_odom_pose is not None:
            # Store robots pose for SLAM
            self.robot_poses.append(current_odom_pose)
            
            # Periodically optimize pose graph
            current_time = rospy.Time.now()
            if (current_time - self.last_optimization_time) > self.optimization_interval:
                self.optimize_pose_graph()
                self.last_optimization_time = current_time
        
        self.last_odom_pose = current_odom_pose

   
    def update_visualization(self):
        """Update visualization markers"""
        # Clear existing markers
        self.marker_array.markers = []
        
        # Add markers for AprilTags
        for tag_id, poses in self.tag_poses.items():
            if poses:
                marker = self.create_marker(tag_id, poses[-1], ColorRGBA(1.0, 0.0, 0.0, 1.0))
                self.marker_array.markers.append(marker)
        
        # Publish markers
        self.marker_pub.publish(self.marker_array)

    #For working with Rviz
    def create_marker(self, tag_id, pose, color):
        """Create a visualization marker for an AprilTag"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "apriltags"
        marker.id = self.marker_id
        self.marker_id += 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color = color
        return marker

if __name__ == "__main__":
    try:
        node = IntegratedDuckieController()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down Integrated Duckie Controller.")

