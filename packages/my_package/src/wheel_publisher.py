#!/usr/bin/env python3

import os
import time
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import Bool, String

class WheelController(DTROS):
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(WheelController, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        # Get vehicle name from environment
        self._vehicle_name = os.environ['VEHICLE_NAME']
        
        # Publishers
        self.cmd_pub = self.publisher(
            f"/{self._vehicle_name}/car_cmd_switch_node/cmd", 
            Twist2DStamped, 
            queue_size=1
        )
        self.led_pub = self.publisher(
            f"/{self._vehicle_name}/leds", 
            String, 
            queue_size=1
        )
        
        # Subscriber
        self.sub = self.subscriber(
            f"/{self._vehicle_name}/red_line", 
            Bool, 
            self.red_line_callback
        )
        
        # Initial state
        self.linear_vel = 0.2
        self.angular_vel = 0.0
        self.stop_requested = False
        self.stop_time = None
        self.saved_vel = 0.0

    def red_line_callback(self, msg):
        if msg.data and not self.stop_requested:
            self.loginfo("Stopping for red line...")
            self.stop_requested = True
            self.stop_time = time.time()
            self.saved_vel = self.linear_vel
            self.linear_vel = 0.0
            self.led_pub.publish("red")

    def on_shutdown(self):
        # Stop the robot when shutting down
        cmd = Twist2DStamped()
        cmd.v = 0.0
        cmd.omega = 0.0
        self.cmd_pub.publish(cmd)

    def run(self):
        rate = self.rate(20)
        while not self.is_shutdown:
            # Handle stop sequence
            if self.stop_requested and (time.time() - self.stop_time >= 4.0):
                self.loginfo("Resuming after red line")
                self.linear_vel = self.saved_vel
                self.led_pub.publish("green")
                self.stop_requested = False

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
    controller.run()
