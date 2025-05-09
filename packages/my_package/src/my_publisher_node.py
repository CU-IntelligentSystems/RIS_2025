#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
from std_msgs.msg import Float32  # Used for subscribing to detection status

class EncoderPIDNode(DTROS):
    def __init__(self, node_name):
        # Initialize DTROS base class with node type
        super(EncoderPIDNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # Get vehicle name from environment and build relevant topic strings
        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = "/pid/cmd_vel"  # Output topic to be fed into mux node
        self.publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        self.left_encoder_topic = f"/{vehicle_name}/left_wheel_encoder_node/tick"
        self.right_encoder_topic = f"/{vehicle_name}/right_wheel_encoder_node/tick"

        # Tick storage for encoders
        self.ticks_left = None
        self.ticks_right = None
        self.last_tick_left = None
        self.last_tick_right = None

        # PID control parameters — adjust these to tune response
        self.Kp = 0.005
        self.Ki = 0.0001
        self.Kd = 0.001

        # PID state variables
        self.integral = 0
        self.last_error = 0

        # Base speed to drive forward in a straight line
        self.base_speed = 0.4  # Tune based on robot's motor strength

        # Reset flag to manually clear encoder values
        self.reset_encoders = False

        # Detection flag: stop movement if an object is detected
        self.object_detected = False

        # Subscribe to left and right wheel encoders
        rospy.Subscriber(self.left_encoder_topic, WheelEncoderStamped, self.callback_left)
        rospy.Subscriber(self.right_encoder_topic, WheelEncoderStamped, self.callback_right)

        # Subscribe to object detection status (1 = object ahead, 0 = clear path)
        rospy.Subscriber('/object_detection_status', Float32, self.detection_callback)

    def callback_left(self, data):
        """
        Callback for left wheel encoder. Optionally resets encoder.
        """
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
        """
        Callback for right wheel encoder. Optionally resets encoder.
        """
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
        """
        Callback for object detection — 1.0 if object detected, else 0.0
        """
        self.object_detected = (msg.data == 1.0)

    def reset_encoder_ticks(self):
        """
        Public method to trigger a reset of both encoder values.
        Useful if called via a service or other script.
        """
        self.reset_encoders = True
        rospy.loginfo("Encoder ticks will be reset on next callback.")

    def run(self):
        """
        Main control loop: runs at 10 Hz, applies PID logic only if object is not detected.
        """
        rospy.loginfo("Running PID controller (mux input mode). Press Ctrl+C to stop.")
        rate = rospy.Rate(10)  # Control loop at 10 Hz

        while not rospy.is_shutdown():
            # Stop sending velocity commands if object detected
            if self.object_detected:
                rospy.loginfo("Object detected. PID paused (no command sent).")
            else:
                # Proceed only if both encoders have valid data
                if self.ticks_left is not None and self.ticks_right is not None:
                    # Calculate encoder error (imbalance)
                    error = self.ticks_left - self.ticks_right

                    # PID calculation
                    self.integral += error
                    derivative = error - self.last_error
                    correction = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

                    # Apply correction to left/right wheel speeds
                    left_cmd = self.base_speed - correction
                    right_cmd = self.base_speed + correction

                    # Clamp wheel speeds to robot's range [-1.0, 1.0]
                    left_cmd = max(min(left_cmd, 1.0), -1.0)
                    right_cmd = max(min(right_cmd, 1.0), -1.0)

                    # Publish corrected wheel speeds
                    msg = WheelsCmdStamped()
                    msg.vel_left = left_cmd
                    msg.vel_right = right_cmd
                    self.publisher.publish(msg)

                    # Logging for debugging
                    rospy.loginfo(f"[ENCODERS] Left: {self.ticks_left}, Right: {self.ticks_right}")
                    rospy.loginfo(f"[CMD] L: {left_cmd:.3f}, R: {right_cmd:.3f} | Error: {error} | Correction: {correction:.4f}")

                    # Update last error
                    self.last_error = error
                else:
                    rospy.loginfo("Waiting for encoder data...")

            rate.sleep()

        # Safety: stop wheels on shutdown
        self.publisher.publish(WheelsCmdStamped(vel_left=0.0, vel_right=0.0))
        rospy.loginfo("PID controller node stopped.")

if __name__ == '__main__':
    # Create and run the encoder-based PID controller node
    node = EncoderPIDNode(node_name='encoder_pid_node')
    node.run()
