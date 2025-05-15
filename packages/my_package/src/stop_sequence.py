#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Float32

class AccelerationNode(DTROS):
    def __init__(self, node_name):
        # Initialize DTROS base class
        super(AccelerationNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        # Get vehicle name from environment and define the command output topic
        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = "/stop/cmd_vel"  # This feeds into the mux for stopping behavior
        self.publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        # Subscribe to object distance only (no detection flag anymore)
        rospy.Subscriber("/object_distance", Float32, self.distance_callback)

        # Motion control parameters
        self.max_speed = 0.6         # Top speed when far away
        self.min_speed = 0.1         # Minimum cautious speed
        self.stop_distance = 0.15    # Stop threshold (in meters)

        self.current_distance = None

    def distance_callback(self, msg):
        """
        Updates the current distance to the detected object.
        """
        self.current_distance = msg.data
        # rospy.loginfo(f"Received distance: {self.current_distance:.2f}m")

    def set_speed(self, left_speed, right_speed):
        """
        Sends wheel speed commands to the robot.
        """
        msg = WheelsCmdStamped()
        msg.vel_left = left_speed
        msg.vel_right = right_speed
        self.publisher.publish(msg)

    def adjust_speed_based_on_distance(self):
        """
        Adjusts speed only based on object distance.
        """
        if self.current_distance is None:
            return  # Skip if no data

        if self.current_distance < self.stop_distance:
            # Too close — stop immediately
            rospy.loginfo("Too close! Stopping.")
            self.set_speed(0.0, 0.0)
        else:
            # Linearly scale speed between stop_distance and 0.5 m
            distance_range = max(0.01, 0.5 - self.stop_distance)
            scale = (self.current_distance - self.stop_distance) / distance_range
            speed = self.min_speed + scale * (self.max_speed - self.min_speed)
            speed = max(self.min_speed, min(speed, self.max_speed))
            rospy.loginfo(f"Safe distance. Moving at speed: {speed:.2f}")
            self.set_speed(speed, speed)

    def run(self):
        """
        Main control loop.
        """
        rospy.loginfo("Starting AccelerationNode loop...")
        rate = rospy.Rate(10)  # 10 Hz loop rate

        while not rospy.is_shutdown():
            self.adjust_speed_based_on_distance()
            rate.sleep()

if __name__ == '__main__':
    node = AccelerationNode(node_name='acceleration_deceleration')
    node.run()
