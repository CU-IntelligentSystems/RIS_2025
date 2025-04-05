#!/usr/bin/env python3

import os
import rospy
import time
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped

class WheelControlNode(DTROS):
    def __init__(self, node_name):
        super(WheelControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        
    def set_speed(self, left_speed, right_speed, duration):
        """Sets wheel speed for a specific duration."""
        message = WheelsCmdStamped(vel_left=left_speed, vel_right=right_speed)
        start_time = time.time()
        rate = rospy.Rate(10)  # 10 Hz publishing
        
        while not rospy.is_shutdown() and (time.time() - start_time) < duration:
            self._publisher.publish(message)
            rate.sleep()
        
    def run(self):
        rospy.loginfo("Starting speed sequence...")
        
        # 30% speed for 5 seconds
        self.set_speed(0.3, 0.3, 5)
        
        # 60% speed for 10 seconds
        self.set_speed(0.6, 0.6, 10)
        
        # 30% speed for 5 seconds
        self.set_speed(0.3, 0.3, 5)
        
        # Stop the robot
        self.set_speed(0.0, 0.0, 1)
        rospy.loginfo("Speed sequence completed.")

if __name__ == '__main__':
    node = WheelControlNode(node_name='wheel_control_node')
    node.run()
