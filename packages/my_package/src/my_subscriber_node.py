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
        wheels_topic = "/stop/cmd_vel"  # This topic feeds into the mux as a stopping/override input
        self.publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        # Subscribe to object distance (from camera node) and detection status
        rospy.Subscriber("/object_distance", Float32, self.distance_callback)
        rospy.Subscriber("/object_detection_status", Float32, self.detection_status_callback)

        # Motion control parameters — tweak these as needed
        self.max_speed = 0.6         # Top speed when nothing is nearby
        self.min_speed = 0.1         # Minimum speed when cautiously approaching
        self.stop_distance = 0.15    # Distance threshold to trigger a full stop (in meters)

        # Runtime state variables
        self.current_distance = None
        self.object_detected = False  # Will be updated by detection_status_callback

    def distance_callback(self, msg):
        """
        Updates the current distance to the detected object.
        """
        self.current_distance = msg.data
        # Debugging optional
        # rospy.loginfo(f"Received distance: {self.current_distance:.2f}m")

    def detection_status_callback(self, msg):
        """
        Updates the detection status — called when red object is seen.
        """
        self.object_detected = (msg.data == 1.0)
        rospy.loginfo(f"[STOP NODE] Object detected: {self.object_detected}")

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
        Adjusts the speed of the Duckiebot based on object distance.
        If object is too close, the robot stops. Otherwise, it slows down based on proximity.
        """
        if self.object_detected:
            if self.current_distance is not None and self.current_distance < self.stop_distance:
                # Too close — stop immediately
                rospy.loginfo("Object too close! Stopping robot.")
                self.set_speed(0.0, 0.0)
            else:
                # Object is detected but not too close — move cautiously
                if self.current_distance is not None:
                    # Linearly scale speed from min to max based on distance
                    speed = max(self.min_speed, (self.current_distance - self.stop_distance) * (self.max_speed / 0.5))
                    speed = min(speed, self.max_speed)
                else:
                    # If distance unknown, use a conservative default
                    speed = self.max_speed

                rospy.loginfo(f"Object detected. Moving cautiously at speed: {speed:.2f}")
                self.set_speed(speed, speed)
        else:
            # No object detected — don't interfere; let PID node take control via mux
            pass

    def run(self):
        """
        Main loop that continuously adjusts speed based on detection input.
        """
        rospy.loginfo("Starting stop node loop...")
        rate = rospy.Rate(10)  # Loop at 10 Hz

        while not rospy.is_shutdown():
            self.adjust_speed_based_on_distance()
            rate.sleep()

if __name__ == '__main__':
    # Create and run the acceleration-deceleration node
    node = AccelerationNode(node_name='acceleration_deceleration')
    node.run()
