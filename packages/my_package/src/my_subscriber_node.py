#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Float32  # Import message type for distance

class AccelerationNode(DTROS):
    def __init__(self, node_name):
        super(AccelerationNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        # Subscribe to /object_distance topic
        rospy.Subscriber("/object_distance", Float32, self.distance_callback)

        # Define speed limits
        self.max_speed = 0.6
        self.min_speed = 0.1  # Smallest speed before stopping
        self.stop_distance = 0.15  # Stop when object is closer than 15 cm

        self.current_distance = None  # Initialize distance variable

    def distance_callback(self, msg):
        """Callback function to update the object's distance."""
        self.current_distance = msg.data
        rospy.loginfo(f"Received distance: {self.current_distance:.2f}m")

    def set_speed(self, left_speed, right_speed):
        """Publish wheel speed commands."""
        message = WheelsCmdStamped()
        message.vel_left = left_speed
        message.vel_right = right_speed
        self._publisher.publish(message)

    def adjust_speed_based_on_distance(self):
        """
        Adjusts the robot's speed smoothly based on the distance to an object.
        The speed decreases gradually as the robot approaches the object.
        """
        if self.current_distance is None:
            rospy.logwarn("No distance data received yet. Stopping robot.")
            self.set_speed(0.0, 0.0)
            return

        if self.current_distance < self.stop_distance:
            rospy.loginfo("Object too close! Stopping...")
            self.set_speed(0.0, 0.0)  # Stop smoothly
        else:
            # Linearly interpolate speed based on distance
            speed = max(self.min_speed, (self.current_distance - self.stop_distance) * (self.max_speed / 0.5))  # Normalize within 50 cm
            rospy.loginfo(f"Distance: {self.current_distance:.2f}m, Speed: {speed:.2f}")
            self.set_speed(speed, speed)

    def run(self):
        rospy.loginfo("Starting robot control loop...")
        rate = rospy.Rate(10)  # 10 Hz loop

        while not rospy.is_shutdown():
            self.adjust_speed_based_on_distance()
            rate.sleep()

if __name__ == '__main__':
    node = AccelerationNode(node_name='acceleration_deceleration')
    node.run()
