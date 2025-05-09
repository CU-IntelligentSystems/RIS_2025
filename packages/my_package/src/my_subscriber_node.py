#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Float32

class AccelerationNode(DTROS):
    def __init__(self, node_name):
        super(AccelerationNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = "/stop/cmd_vel"
        self.publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        # Subscribe to distance and detection status topics
        rospy.Subscriber("/object_distance", Float32, self.distance_callback)
        rospy.Subscriber("/object_detection_status", Float32, self.detection_status_callback)

        # Motion parameters
        self.max_speed = 0.6
        self.min_speed = 0.1
        self.stop_distance = 0.15

        self.current_distance = None
        self.object_detected = False  # Track whether an object is detected

    def distance_callback(self, msg):
        self.current_distance = msg.data
        # rospy.loginfo(f"Received distance: {self.current_distance:.2f}m")  # Optional: keep or remove

    def detection_status_callback(self, msg):
        self.object_detected = (msg.data == 1.0)
        rospy.loginfo(f"[STOP NODE] Object detected: {self.object_detected}")

    def set_speed(self, left_speed, right_speed):
        msg = WheelsCmdStamped()
        msg.vel_left = left_speed
        msg.vel_right = right_speed
        self.publisher.publish(msg)

    def adjust_speed_based_on_distance(self):
        if self.object_detected:
            if self.current_distance is not None and self.current_distance < self.stop_distance:
                rospy.loginfo("Object too close! Stopping robot.")
                self.set_speed(0.0, 0.0)
            else:
                # Adjust speed based on distance, or use default if unknown
                if self.current_distance is not None:
                    speed = max(self.min_speed, (self.current_distance - self.stop_distance) * (self.max_speed / 0.5))
                    speed = min(speed, self.max_speed)
                else:
                    speed = self.max_speed

                rospy.loginfo(f"Object detected. Moving cautiously at speed: {speed:.2f}")
                self.set_speed(speed, speed)
        else:
            # Do nothing – let the PID node control the robot via mux
            pass

    def run(self):
        rospy.loginfo("Starting stop node loop...")
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.adjust_speed_based_on_distance()
            rate.sleep()

if __name__ == '__main__':
    node = AccelerationNode(node_name='acceleration_deceleration')
    node.run()
