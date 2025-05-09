#!/usr/bin/env python3

import os
import rospy
import time
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from camera_reader_node import distance 

class Acceleration(DTROS):
    def __init__(self, node_name):
        super(AccelerationNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        
    def set_speed(self, left_speed, right_speed, distance):
        """Change the wheel speed throught the distance to create a gradual acceleration."""
        message = WheelsCmdStamped(vel_left=left_speed, vel_right=right_speed)
        self._publisher.publish(message)
        


        # start_time = time.time()
        # rate = rospy.Rate(10)  # 10 Hz publishing
        
        # while not rospy.is_shutdown() and (time.time() - start_time) * a + min_speed < max_speed:
        #     self._publisher.publish(message)
        #     rate.sleep()
        
    def run(self):
        rospy.loginfo("Starting speed sequence...")
        
        max_speed = 0.6
        min_speed = 0.3
        distance = 50 #cm

        start_time = time.time()
        rate = rospy.Rate(10)  # 10 Hz publishing

        a = (max_speed**2 - min_speed**2) // (2 * distance) # acceleration

        while not rospy.is_shutdown() and (time.time() - start_time) * a + min_speed < max_speed:
            self.set_speed(min_speed + (time.time() - start_time) * a, min_speed + (time.time() - start_time) * a, distance)


        # # 60% speed for 10 seconds
        # self.set_speed(0.6, 0.6, 10)
        
        # # 30% speed for 5 seconds
        # self.set_speed(0.3, 0.3, 5)
        
        # # Stop the robot
        # self.set_speed(0.0, 0.0, 1)
        # rospy.loginfo("Speed sequence completed.")

if __name__ == '__main__':
    node = Acceleration(node_name='acceleration_deceleration')
    node.run()
