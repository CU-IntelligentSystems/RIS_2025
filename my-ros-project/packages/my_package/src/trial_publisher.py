#!/usr/bin/env python3

import os
import rospy
import numpy as np
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class WheelCommandPublisherNode(DTROS):

    def __init__(self, node_name):
        super(WheelCommandPublisherNode, self).__init__(
            node_name=node_name, 
            node_type=NodeType.CONTROL
        )
        
        # Get vehicle name from environment
        self._vehicle_name = os.environ['VEHICLE_NAME']
        
        # Create command publisher
        self._cmd_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        self._publisher = rospy.Publisher(
            self._cmd_topic, 
            Twist2DStamped, 
            queue_size=1
        )
        
        # Camera setup
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._bridge = CvBridge()
        self._window = "Camera Viewer"
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        self.sub_camera = rospy.Subscriber(
            self._camera_topic,
            CompressedImage,
            self.camera_callback
        )
        
        # Command state
        self._command = Twist2DStamped()
        self._command.header.seq = 0
        self._command.header.stamp = rospy.Time.now()
        self._command.header.frame_id = "base_link"
        
        # Movement states
        self._linear_velocity = 0.2  # Default forward speed
        self._angular_velocity = 0.0  # No turn
        self.stop_requested = False
        self.stop_start_time = None
        self.saved_velocity = 0.0
        self.saved_angular = 0.0

    def camera_callback(self, msg):
        try:
            # Convert image
            image = self._bridge.compressed_imgmsg_to_cv2(msg)
            
            # Display frame
            cv2.imshow(self._window, image)
            cv2.waitKey(1)
            
            # Red line detection logic (replace with your implementation)
            # Example: Check for red pixels in lower third of image
            height, width, _ = image.shape
            roi = image[int(height*0.6):height, :]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            lower_red = np.array([0, 100, 100])
            upper_red = np.array([10, 255, 255])
            mask = cv2.inRange(hsv, lower_red, upper_red)
            red_pixels = np.sum(mask > 0)
            
            if red_pixels > 10000:  # Threshold for detection
                self.stop_requested = True
                self.stop_start_time = rospy.get_time()
                self.saved_velocity = self._linear_velocity
                self.saved_angular = self._angular_velocity
                self._linear_velocity = 0.0
                self._angular_velocity = 0.0
                rospy.loginfo("Red line detected! Initiating stop sequence.")
                
        except Exception as e:
            rospy.logerr(f"Camera processing error: {e}")

    def run(self):
        rate = rospy.Rate(20)  # 20Hz
        
        while not rospy.is_shutdown():
            current_time = rospy.get_time()
            
            # Handle stop sequence
            if self.stop_requested and (current_time - self.stop_start_time) >= 4.0:
                self._linear_velocity = self.saved_velocity
                self._angular_velocity = self.saved_angular
                self.stop_requested = False
                rospy.loginfo("Resuming lane following.")

            # Update message header
            self._command.header.seq += 1
            self._command.header.stamp = rospy.Time.now()
            
            # Set velocities
            self._command.v = self._linear_velocity
            self._command.omega = self._angular_velocity
            
            # Publish command
            self._publisher.publish(self._command)
            
            rate.sleep()

    def stop(self):
        self._command.v = 0.0
        self._command.omega = 0.0
        self._publisher.publish(self._command)

if __name__ == '__main__':
    node = WheelCommandPublisherNode(node_name='wheel_command_publisher_node')
    
    try:
        # Start with default forward motion
        node.set_velocity(linear=0.2, angular=0.0)
        node.run()
    except rospy.ROSInterruptException:
        node.stop()
