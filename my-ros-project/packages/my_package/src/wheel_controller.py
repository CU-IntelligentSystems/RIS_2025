#!/usr/bin/env python3

# This code controls the DuckieBot's wheels, making it stop for 4 seconds when a redline is detected, and also publishes LED color commands to indicate the robot's state - red when stopped, green when moving.

import os
import time
import rospy
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import Bool, String

class WheelController:
    def __init__(self, node_name):  # constructor
        # Initialize the ROS node with the provided name
        rospy.init_node(node_name, anonymous=True)
        # Get vehicle name from environment
        self._vehicle_name = os.environ['VEHICLE_NAME']  # Must be set in environment
        # Publishers
        self.cmd_pub = rospy.Publisher(
            f"/{self._vehicle_name}/car_cmd_switch_node/cmd", 
            Twist2DStamped,  # message type for velocity commands
            queue_size=1
        )
        self.led_pub = rospy.Publisher(
            f"/{self._vehicle_name}/leds", 
            String, 
            queue_size=1
        )
        # Subscriber
        self.sub = rospy.Subscriber(
            f"/{self._vehicle_name}/red_line", 
            Bool, 
            self.red_line_callback
        )
        # Initial state
        self.linear_vel = 0.2  # duckiebot moves forward at this speed
        self.angular_vel = 0.0  # angular velocity to 0, i.e. no turning
        self.stop_requested = False  # a flag to track whether the DuckieBot should stop.. initially set to false
        self.stop_time = None  # tracks the time when stop is requested
        self.saved_vel = 0.0  # stores linear velocity before stopping so that it can be restored later
        # Register shutdown hook
        rospy.on_shutdown(self.on_shutdown)

    def red_line_callback(self, msg):
        if msg.data and not self.stop_requested:
            rospy.loginfo("Stopping for red line...")
            self.stop_requested = True
            self.stop_time = time.time()  # records the current time to track the 4-second stop duration
            self.saved_vel = self.linear_vel
            self.linear_vel = 0.0  # sets linear velocity to 0, stopping the bot
            self.led_pub.publish("red")  # publishes a "red" message, triggering the LEDs to turn red

    def on_shutdown(self):
        # Stop the robot when shutting down
        rospy.loginfo("Shutting down wheel_controller, stopping the robot...")
        cmd = Twist2DStamped()
        cmd.v = 0.0
        cmd.omega = 0.0
        self.cmd_pub.publish(cmd)  # publishes the command to stop the DuckieBot
        rospy.sleep(0.5)  # ensure the message is sent before shutdown

    def run(self):
        rate = rospy.Rate(20)  # sets loop rate to 20 iterations per second
        while not rospy.is_shutdown():
            # Handle stop sequence
            if self.stop_requested and (time.time() - self.stop_time >= 4.0):  # checks if the bot has stopped and 4 seconds have passed
                rospy.loginfo("Resuming after red line")
                self.linear_vel = self.saved_vel  # restores original velocity
                self.led_pub.publish("green")  # publishes a "green" message to LEDs to turn green - bot is now moving
                self.stop_requested = False  # resetting the stop flag
            # Create and publish command
            cmd = Twist2DStamped()
            cmd.v = self.linear_vel
            cmd.omega = self.angular_vel
            self.cmd_pub.publish(cmd)
            rate.sleep()

if __name__ == '__main__':
    # Create the node
    controller = WheelController(node_name='wheel_controller')
    # Run the node
    try:
        controller.run()
    except rospy.ROSInterruptException:
        pass
