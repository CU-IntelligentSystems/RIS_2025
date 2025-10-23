#!/usr/bin/env python3

# This node implements a PID controller that uses left and right wheel encoder feedback
# to keep a Duckiebot driving in a straight line. It publishes corrected wheel velocities
# to /pid/cmd_vel, which are selected by the twist_mux system unless overridden.

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped

class EncoderPIDNode(DTROS):
    def __init__(self, node_name):
        # Initialize the DTROS node with the specified name and type
        super(EncoderPIDNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # Retrieve vehicle name from environment variable and define publishing topic
        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = "/pid/cmd_vel"
        self.publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        # Define encoder topic names
        self.left_encoder_topic = f"/{vehicle_name}/left_wheel_encoder_node/tick"
        self.right_encoder_topic = f"/{vehicle_name}/right_wheel_encoder_node/tick"

        # Encoder tick storage
        self.ticks_left = None
        self.ticks_right = None
        self.last_tick_left = None
        self.last_tick_right = None

        # PID control gains (tunable)
        self.Kp = 0.005
        self.Ki = 0.0001
        self.Kd = 0.001

        # PID internal state
        self.integral = 0
        self.last_error = 0
        self.last_time = rospy.Time.now().to_sec()

        # Base driving speed (robot moves forward at this speed when balanced)
        self.base_speed = 0.4

        # Reset flag for encoder calibration
        self.reset_encoders = False

        # Subscribe to encoder tick topics
        rospy.Subscriber(self.left_encoder_topic, WheelEncoderStamped, self.callback_left)
        rospy.Subscriber(self.right_encoder_topic, WheelEncoderStamped, self.callback_right)

    def callback_left(self, data):
        """
        Handles incoming left wheel encoder ticks.
        Supports reset if flagged.
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
        Handles incoming right wheel encoder ticks.
        Supports reset if flagged.
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

    def reset_encoder_ticks(self):
        """
        External call to request reset of both encoders.
        """
        self.reset_encoders = True
        rospy.loginfo("Encoder ticks will be reset on next callback.")

    def run(self):
        """
        Main control loop.
        Executes PID calculation at 10 Hz and publishes corrected wheel speeds.
        """
        rospy.loginfo("Running PID controller. Press Ctrl+C to stop.")
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # Ensure we have valid encoder data before proceeding
            if self.ticks_left is not None and self.ticks_right is not None:
                # Compute tick deltas since last loop
                delta_left = self.ticks_left - self.last_tick_left
                delta_right = self.ticks_right - self.last_tick_right

                # Update previous tick values
                self.last_tick_left = self.ticks_left
                self.last_tick_right = self.ticks_right

                # Calculate PID error as difference in movement between wheels
                error = delta_left - delta_right
                self.integral += error
                self.integral = max(min(self.integral, 100), -100)  # Prevent integral windup

                # Calculate time delta
                current_time = rospy.Time.now().to_sec()
                dt = current_time - self.last_time
                derivative = (error - self.last_error) / dt if dt > 0 else 0
                self.last_time = current_time

                # PID correction formula
                correction = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

                # Apply correction to base speed for left and right wheels
                left_cmd = self.base_speed - correction
                right_cmd = self.base_speed + correction

                # Clamp speed values within motor bounds
                left_cmd = max(min(left_cmd, 1.0), -1.0)
                right_cmd = max(min(right_cmd, 1.0), -1.0)

                # Publish wheel command
                msg = WheelsCmdStamped()
                msg.vel_left = left_cmd
                msg.vel_right = right_cmd
                self.publisher.publish(msg)

                # Log for debugging and analysis
                rospy.loginfo(f"[ENC DELTA] L: {delta_left}, R: {delta_right}")
                rospy.loginfo(f"[CMD] L: {left_cmd:.3f}, R: {right_cmd:.3f} | E: {error} | C: {correction:.4f}")
                self.last_error = error
            else:
                rospy.loginfo("Waiting for encoder data...")

            rate.sleep()

        # Stop wheels when node is shutting down
        self.publisher.publish(WheelsCmdStamped(vel_left=0.0, vel_right=0.0))
        rospy.loginfo("PID controller node stopped.")

if __name__ == '__main__':
    node = EncoderPIDNode(node_name='encoder_pid_node')
    node.run()
