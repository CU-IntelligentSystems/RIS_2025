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

        rate = rospy.Rate(10)  # 10 Hz publishing

        self._publisher.publish(message)
        rate.sleep()


        # start_time2 = time.time()
        # rate = rospy.Rate(10)  # 10 Hz publishing
        
        # while not rospy.is_shutdown() and (time.time() - start_time2) < duration:
        #     self._publisher.publish(message)
        #     rate.sleep()

    def run(self):
        rospy.loginfo("Starting speed sequence...")
        

        # self.set_speed(0.0, 0.0, 1)
         
        max_speed = 0.7
        min_speed = 0.2
        distance = 1 #m

        #idk it is dviitng by 3

        prev_time = time.time()
        # rate = rospy.Rate(10)  # 10 Hz publishing
        
        a = (max_speed**2 - min_speed**2) / (2 * distance) # acceleration

        print("This is acceleration - ", a)

        speed = min_speed

        # 30% speed for 5 seconds
        # self.set_speed(speed, speed, 2)
        print("SPEED", speed)


        while not rospy.is_shutdown() and speed < max_speed:

            now = time.time()
            dt = now - prev_time
            prev_time = now
            
            speed += dt * a

            print(speed, dt)
            self.set_speed(speed, speed, 0.1)
            # rate = rospy.Rate(10)  # 10 Hz publishing

        print("SPEED", speed)

        # 30% speed for 5 seconds
        # self.set_speed(speed, speed, 2)
        
        # Stop the robot
        self.set_speed(0.0, 0.0, 1)
        rospy.loginfo("Speed sequence completed.")

if __name__ == '__main__':
    node = WheelControlNode(node_name='wheel_control_node')
    node.run()
