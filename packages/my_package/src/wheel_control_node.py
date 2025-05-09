#!/usr/bin/env python3

import os
import rospy
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA

class LEDColorCycleNode:
    def __init__(self):
        rospy.init_node("led_color_cycle_node", anonymous=False)

        vehicle_name = os.environ['VEHICLE_NAME']
        pattern_topic = f"/{vehicle_name}/led_emitter_node/led_pattern"
        self.led_pattern_pub = rospy.Publisher(pattern_topic, LEDPattern, queue_size=1)

        rospy.sleep(1.0)  # Allow time for publisher to connect

        self.colors = [
            ColorRGBA(1.0, 0.0, 0.0, 1.0),  # Red
            ColorRGBA(0.0, 1.0, 0.0, 1.0),  # Green
            ColorRGBA(0.0, 0.0, 1.0, 1.0),  # Blue
            ColorRGBA(1.0, 1.0, 1.0, 1.0),  # White
            ColorRGBA(1.0, 1.0, 0.0, 1.0)   # Yellow
        ]

        self.run_cycle()

    def send_led_pattern(self, color):
        msg = LEDPattern()
        msg.rgb_vals = [color] * 5  # All LEDs to same color
        msg.frequency = 0.0         # No blinking
        msg.frequency_mask = [0] * 5
        self.led_pattern_pub.publish(msg)
        rospy.loginfo(f"Sent LED pattern: {color}")

    def run_cycle(self):
        rate = rospy.Rate(1)  # 1 Hz = 1 color per second
        while not rospy.is_shutdown():
            for color in self.colors:
                self.send_led_pattern(color)
                rate.sleep()

if __name__ == "__main__":
    try:
        LEDColorCycleNode()
    except rospy.ROSInterruptException:
        pass
