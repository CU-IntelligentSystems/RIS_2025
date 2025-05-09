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
        wheels_topic = "/pid/cmd_vel"  # Publish to mux input topic
        self.publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        self.left_encoder_topic = f"/{vehicle_name}/left_wheel_encoder_node/tick"
        self.right_encoder_topic = f"/{vehicle_name}/right_wheel_encoder_node/tick"

        self.ticks_left = None
        self.ticks_right = None
        self.last_tick_left = None
        self.last_tick_right = None

        # PID gains
        self.Kp = 0.005
        self.Ki = 0.0001
        self.Kd = 0.001

        self.integral = 0
        self.last_error = 0

        self.base_speed = 0.4

        # Flag to control encoder reset
        self.reset_encoders = False
        self.object_detected = False  # Assume no object detected initially

        # Subscribers
        rospy.Subscriber(self.left_encoder_topic, WheelEncoderStamped, self.callback_left)
        rospy.Subscriber(self.right_encoder_topic, WheelEncoderStamped, self.callback_right)
        rospy.Subscriber('/object_detection_status', Float32, self.detection_callback)

    def callback_left(self, data):
        if self.reset_encoders:
            self.ticks_left = 0
            self.last_tick_left = 0
            self.reset_encoders = False
            rospy.loginfo("Left encoder reset.")
        else:
            self.ticks_left = data.data
            if self.last_tick_left is None:
                self.last_tick_left = self.ticks_left

    def callback_right(self, data):
        if self.reset_encoders:
            self.ticks_right = 0
            self.last_tick_right = 0
            self.reset_encoders = False
            rospy.loginfo("Right encoder reset.")
        else:
            self.ticks_right = data.data
            if self.last_tick_right is None:
                self.last_tick_right = self.ticks_right

    def detection_callback(self, msg):
        self.object_detected = (msg.data == 1.0)

    def reset_encoder_ticks(self):
        self.reset_encoders = True
        rospy.loginfo("Encoder ticks will be reset on next callback.")

    def run(self):
        rospy.loginfo("Running PID controller (mux input mode). Press Ctrl+C to stop.")
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            if not self.object_detected:
                if self.ticks_left is not None and self.ticks_right is not None:
                    error = self.ticks_left - self.ticks_right

                    # PID computation
                    self.integral += error
                    derivative = error - self.last_error
                    correction = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

                    left_cmd = self.base_speed - correction
                    right_cmd = self.base_speed + correction

                    left_cmd = max(min(left_cmd, 1.0), -1.0)
                    right_cmd = max(min(right_cmd, 1.0), -1.0)

                    msg = WheelsCmdStamped()
                    msg.vel_left = left_cmd
                    msg.vel_right = right_cmd
                    self.publisher.publish(msg)

                    rospy.loginfo(f"[ENCODERS] Left: {self.ticks_left}, Right: {self.ticks_right}")
                    rospy.loginfo(f"[CMD] L: {left_cmd:.3f}, R: {right_cmd:.3f} | Error: {error} | Correction: {correction:.4f}")

                    self.last_error = error
                else:
                    rospy.loginfo("Waiting for encoder data...")
            else:
                rospy.loginfo("Object detected. PID paused (no command sent).")

            rate.sleep()

        # Stop wheels on shutdown
        self.publisher.publish(WheelsCmdStamped(vel_left=0.0, vel_right=0.0))
        rospy.loginfo("PID controller node stopped.")

if __name__ == '__main__':
    node = EncoderPIDNode(node_name='encoder_pid_node')
    node.run()
