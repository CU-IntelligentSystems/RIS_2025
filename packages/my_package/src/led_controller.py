#!/usr/bin/env python3

import rospy
import os
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import String

class LEDController:
    def __init__(self, node_name):
        # Initialize the ROS node with the provided name
        rospy.init_node(node_name, anonymous=True)
        # Get vehicle name from environment
        self._vehicle_name = os.environ['VEHICLE_NAME']  # Must be set in environment
        # Define publisher and subscriber topics
        self.led_pub = rospy.Publisher(
            f"/{self._vehicle_name}/led_emitter_node/led_pattern", 
            LEDPattern, 
            queue_size=1
            # This limits the message queue to 1, meaning that only the latest message is kept if the publisher is fast.
        )
        self.color_sub = rospy.Subscriber(
            f"/{self._vehicle_name}/leds", 
            String, 
            self.color_callback,
            queue_size=10  # Increased to prevent message loss
            # creating a subscriber to listen for string messages like "red" or "green".
        )
        # Debug mode: Toggle colors every 5 seconds
        self.debug_color = "red"  # Start with red
        self.debug_timer = rospy.Timer(rospy.Duration(5.0), self.debug_callback)  # Toggle every 5 seconds

    def color_callback(self, msg):
        try:
            rospy.loginfo(f"Received LED command: {msg.data}")
            pattern = LEDPattern()  # message specifying LED colors
            if msg.data == "red":  # Checking if the received message contains the string "red"
                pattern.rgb_vals = [255, 0, 0] * 4  # All red (adjusted to * 4 because the duckiebot has 4 LEDs)
            elif msg.data == "green":
                pattern.rgb_vals = [0, 255, 0] * 4  # All green (adjust to * 4 because the duckiebot has 4 LEDs)
            else:
                rospy.logwarn(f"Unknown color command: {msg.data}")
                return
            self.led_pub.publish(pattern)  # Publishing the LEDPattern message to the led_pattern topic, instructing the Duckiebot's LEDs to change to the specified color.
            rospy.loginfo(f"Published LED pattern: {pattern.rgb_vals}")
        except Exception as e:
            rospy.logerr(f"Error publishing LED pattern: {e}")

    def debug_callback(self, event):
        # Toggle between colors every 5 seconds to test LEDs
        try:
            pattern = LEDPattern()
            if self.debug_color == "red":
                pattern.rgb_vals = [255, 0, 0] * 4  # Red
                self.debug_color = "green"
            else:
                pattern.rgb_vals = [0, 255, 0] * 4 # Green
                self.debug_color = "red"
            self.led_pub.publish(pattern)
            rospy.loginfo(f"Debug: Published test LED pattern ({self.debug_color}): {pattern.rgb_vals}")
        except Exception as e:
            rospy.logerr(f"Debug: Error publishing test LED pattern: {e}")

if __name__ == '__main__':
    # Create the node
    node = LEDController(node_name='led_controller')
    # Keep spinning
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
