#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
from std_msgs.msg import Float32  # For subscribing to object detection status

class EncoderPIDNode(DTROS):
    def __init__(self, node_name):
        super(EncoderPIDNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        self._left_encoder_topic = f"/{vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{vehicle_name}/right_wheel_encoder_node/tick"

        self._ticks_left = None
        self._ticks_right = None
        self._last_tick_left = None
        self._last_tick_right = None

        # PID gains
        self.Kp = 0.005
        self.Ki = 0.0001
        self.Kd = 0.001

        self.integral = 0
        self.last_error = 0

        self.base_speed = 0.2
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)

        # Subscriber for object detection status
        self.detection_sub = rospy.Subscriber('/object_detection_status', Float32, self.detection_callback)

        self.reset_encoders = False
        self.object_detected = False  # Initially assume no object detected

    def callback_left(self, data):
        if self.reset_encoders:
            self._ticks_left = 0
            self._last_tick_left = 0
            self.reset_encoders = False
        else:
            self._ticks_left = data.data
            if self._last_tick_left is None:
                self._last_tick_left = self._ticks_left

    def callback_right(self, data):
        if self.reset_encoders:
            self._ticks_right = 0
            self._last_tick_right = 0
            self.reset_encoders = False
        else:
            self._ticks_right = data.data
            if self._last_tick_right is None:
                self._last_tick_right = self._ticks_right

    def detection_callback(self, msg):
        # Check if object detection status is 0 (no object detected) or 1 (object detected)
        if msg.data == 0.0:
            self.object_detected = False  # No object detected
        elif msg.data == 1.0:
            self.object_detected = True  # Object detected

    def reset_encoder_ticks(self):
        self.reset_encoders = True
        rospy.loginfo("Encoder ticks reset.")

    def run(self):
        rospy.loginfo("Running PID controller indefinitely. Press Ctrl+C to stop.")
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            if not self.object_detected:  # Only run PID when no object is detected
                if self._ticks_left is not None and self._ticks_right is not None:
                    error = self._ticks_left - self._ticks_right

                    # PID computations
                    self.integral += error
                    derivative = error - self.last_error

                    correction = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

                    left_cmd = self.base_speed - correction
                    right_cmd = self.base_speed + correction

                    left_cmd = max(min(left_cmd, 1.0), -1.0)
                    right_cmd = max(min(right_cmd, 1.0), -1.0)

                    self._publisher.publish(WheelsCmdStamped(vel_left=left_cmd, vel_right=right_cmd))

                    rospy.loginfo(f"[ENCODERS] Left: {self._ticks_left}, Right: {self._ticks_right}")
                    rospy.loginfo(f"[CMD] L: {left_cmd:.3f}, R: {right_cmd:.3f} | Error: {error} | Correction: {correction:.4f}")

                    self.last_error = error
                else:
                    rospy.loginfo("Waiting for encoder data...")
            else:
                # Stop the robot if object is detected
                rospy.loginfo("Object detected, stopping robot.")
                self._publisher.publish(WheelsCmdStamped(vel_left=0.0, vel_right=0.0))

            rate.sleep()

        # Stop the robot when shutting down
        self._publisher.publish(WheelsCmdStamped(vel_left=0.0, vel_right=0.0))
        rospy.loginfo("PID controller stopped.")

if __name__ == '__main__':
    node = EncoderPIDNode(node_name='encoder_pid_node')
    node.run()
