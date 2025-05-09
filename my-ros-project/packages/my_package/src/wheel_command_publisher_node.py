#!/usr/bin/env python3

import os
import rospy
import numpy as np
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import Bool  # Required for boolean messages

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
        
        # Create red line subscriber (INPUT FROM CAMERA NODE)
        self._red_line_topic = f"/{self._vehicle_name}/red_line_detector_node/red_line"
        self._subscriber = rospy.Subscriber(
            self._red_line_topic,
            Bool,
            self.red_line_callback
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

    def red_line_callback(self, data):
        # This is WHERE THE INPUT IS COLLECTED FROM SUBSCRIBER (camera node)
        if data.data:  # If red line detected
            self.stop_requested = True
            self.stop_start_time = rospy.get_time()
            # Save current velocities for resuming
            self.saved_velocity = self._linear_velocity
            self.saved_angular = self._angular_velocity
            # Set velocities to zero
            self._linear_velocity = 0.0
            self._angular_velocity = 0.0
            rospy.loginfo("Red line detected! Initiating stop sequence.")

    def set_velocity(self, linear, angular):
        self._linear_velocity = linear
        self._angular_velocity = angular

    def run(self):
        rate = rospy.Rate(20)  # 20Hz
        
        while not rospy.is_shutdown():
            current_time = rospy.get_time()
            
            # Handle stop sequence
            if self.stop_requested and (current_time - self.stop_start_time) >= 4.0:
                # Resume movement after 4 seconds
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
