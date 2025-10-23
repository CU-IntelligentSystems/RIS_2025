#!/usr/bin/env python3

import rospy
import os
from std_msgs.msg import Float32
from duckietown_msgs.msg import WheelsCmdStamped, LEDPattern
from std_msgs.msg import ColorRGBA

class SimpleMux:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('simple_mux_node')

        # Variables to store the latest incoming commands
        self.cmd_pid = None       # Command from PID node (normal driving)
        self.cmd_stop = None      # Command from stop/acceleration node (e.g., red object ahead)
        self.object_detected = False
        self.current_led_state = None  # Track the LED color to prevent redundant publishing

        # Subscribe to velocity command topics
        rospy.Subscriber('/pid/cmd_vel', WheelsCmdStamped, self.pid_callback)
        rospy.Subscriber('/stop/cmd_vel', WheelsCmdStamped, self.stop_callback)

        # Subscribe to object detection status (1.0 if object ahead, else 0.0)
        rospy.Subscriber('/object_detection_status', Float32, self.detection_callback)

        # Define output topics based on vehicle name
        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        led_topic = f"/{vehicle_name}/led_emitter_node/led_pattern"

        # Publisher for wheels and LEDs
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        self._led_pub = rospy.Publisher(led_topic, LEDPattern, queue_size=1)

        rospy.loginfo(f"[MUX] Publishing wheel commands to: {wheels_topic}")
        rospy.loginfo(f"[MUX] Publishing LED patterns to: {led_topic}")

        # Give time for publishers and subscribers to set up
        rospy.sleep(1.0)

        # Start main control loop
        self.loop()

    def pid_callback(self, msg):
        """
        Store latest message from PID controller (for normal driving).
        """
        self.cmd_pid = msg

    def stop_callback(self, msg):
        """
        Store latest message from Stop/Acceleration node (for object avoidance).
        """
        self.cmd_stop = msg

    def detection_callback(self, msg):
        """
        Update internal flag for object detection status.
        """
        self.object_detected = (msg.data == 1.0)

    def set_led_color(self, color_name):
        """
        Publish LEDPattern message to set Duckiebot's lights to specified color.
        Supported colors: "white", "red"
        """
        msg = LEDPattern()

        # Set color based on name
        if color_name == "white":
            color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        elif color_name == "red":
            color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
        else:
            return  # Unsupported color

        # Set all LEDs to the same color
        msg.rgb_vals = [color] * 5
        msg.frequency = 0.0
        msg.frequency_mask = [0] * 5

        # Publish LED update only if it's a new color
        self._led_pub.publish(msg)
        rospy.loginfo(f"[MUX] Set LED color: {color_name}")
        self.current_led_state = color_name

    def loop(self):
        """
        Main control loop:
        - If an object is detected: publish stop command and turn LEDs red
        - If not: publish PID command and turn LEDs white
        """
        rate = rospy.Rate(10)  # 10 Hz loop rate
        while not rospy.is_shutdown():
            if self.object_detected:
                # Object ahead: stop and show red LEDs
                if self.cmd_stop:
                    self._publisher.publish(self.cmd_stop)
                if self.current_led_state != "red":
                    self.set_led_color("red")
            else:
                # No object: normal PID control and white LEDs
                if self.cmd_pid:
                    self._publisher.publish(self.cmd_pid)
                if self.current_led_state != "white":
                    self.set_led_color("white")
            rate.sleep()

if __name__ == '__main__':
    # Instantiate and run the mux node
    SimpleMux()
