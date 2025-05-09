#!/usr/bin/env python3

import rospy
import os
from std_msgs.msg import Float32
from duckietown_msgs.msg import WheelsCmdStamped, LEDPattern
from std_msgs.msg import ColorRGBA

class SimpleMux:
    def __init__(self):
        rospy.init_node('simple_mux_node')

        self.cmd_pid = None
        self.cmd_stop = None
        self.object_detected = False
        self.current_led_state = None  # Track current LED color

        # Subscribers
        rospy.Subscriber('/pid/cmd_vel', WheelsCmdStamped, self.pid_callback)
        rospy.Subscriber('/stop/cmd_vel', WheelsCmdStamped, self.stop_callback)
        rospy.Subscriber('/object_detection_status', Float32, self.detection_callback)

        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        led_topic = f"/{vehicle_name}/led_emitter_node/led_pattern"

        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        self._led_pub = rospy.Publisher(led_topic, LEDPattern, queue_size=1)

        rospy.loginfo(f"[MUX] Publishing to: {wheels_topic}")
        rospy.loginfo(f"[MUX] LED Publisher: {led_topic}")

        rospy.sleep(1.0)  # Allow pub/sub to initialize

        self.loop()

    def pid_callback(self, msg):
        self.cmd_pid = msg

    def stop_callback(self, msg):
        self.cmd_stop = msg

    def detection_callback(self, msg):
        self.object_detected = (msg.data == 1.0)

    def set_led_color(self, color_name):
        msg = LEDPattern()
        if color_name == "white":
            color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        elif color_name == "red":
            color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
        else:
            return

        msg.rgb_vals = [color] * 5
        msg.frequency = 0.0
        msg.frequency_mask = [0] * 5

        self._led_pub.publish(msg)
        rospy.loginfo(f"[MUX] Set LED color: {color_name}")
        self.current_led_state = color_name

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.object_detected:
                if self.cmd_stop:
                    self._publisher.publish(self.cmd_stop)
                if self.current_led_state != "red":
                    self.set_led_color("red")
            else:
                if self.cmd_pid:
                    self._publisher.publish(self.cmd_pid)
                if self.current_led_state != "white":
                    self.set_led_color("white")
            rate.sleep()

if __name__ == '__main__':
    SimpleMux()
