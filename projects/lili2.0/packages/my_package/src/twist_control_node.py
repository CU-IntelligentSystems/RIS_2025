#!/usr/bin/env python3


import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped


# Twist command for controlling the linear and angular velocity of the frame
VELOCITY = 0.3  # linear vel    , in m/s    , forward (+)
OMEGA = 4.0     # angular vel   , rad/s     , counter clock wise (+)


class TwistControlNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(TwistControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        vehicle_name = os.environ['VEHICLE_NAME']
        twist_topic = f"/{vehicle_name}/car_cmd_switch_node/cmd"
        #twist_topic = f"/{vehicle_name}/white_line_detector_follower/custom_car_cmd"
        # form the message
        self._v = VELOCITY
        self._omega = OMEGA
        # construct publisher
        self._publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)

    def run(self):
        # publish 10 messages every second (10 Hz)
        rate = rospy.Rate(10)
        message = Twist2DStamped(v=self._v, omega=self._omega)
        while not rospy.is_shutdown():
            self._publisher.publish(message)
            rate.sleep()

    def on_shutdown(self):
        stop = Twist2DStamped(v=0.0, omega=0.0)
        self._publisher.publish(stop)

if __name__ == '__main__':
    # create the node
    node = TwistControlNode(node_name='twist_control_node')
    # run node
    node.run()
    # keep the process from terminating
    rospy.spin()

'''
"""

TwistRelayNode

  Subscribes to ~/custom_car_cmd (your P-controller output)

  Republishes to /<VEHICLE_NAME>/car_cmd_switch_node/cmd

  so the Duckiebot actually drives.

"""
 
import os

import rospy

from duckietown.dtros import DTROS, NodeType

from duckietown_msgs.msg import Twist2DStamped
 
class TwistRelayNode(DTROS):

    def __init__(self, node_name):

        super(TwistRelayNode, self).__init__(node_name=node_name,

                                             node_type=NodeType.GENERIC)
 
        # 1) INPUT: your custom_car_cmd (private namespace)

        in_topic = rospy.get_param('~in_topic', 'custom_car_cmd')

        self.sub = rospy.Subscriber(f'~/{in_topic}',

                                    Twist2DStamped,

                                    self.cb_relay,

                                    queue_size=1)
 
        # 2) OUTPUT: the real drive topic

        vehicle_name = os.environ['VEHICLE_NAME']

        out_topic = rospy.get_param('~out_topic',

                                    f'/{vehicle_name}/car_cmd_switch_node/cmd')

        self.pub = rospy.Publisher(out_topic,

                                   Twist2DStamped,

                                   queue_size=1)
 
        rospy.loginfo(f"[TwistRelayNode] listening  →  ~/{in_topic}")

        rospy.loginfo(f"[TwistRelayNode] publishing →  {out_topic}")
 
    def cb_relay(self, msg: Twist2DStamped):

        """Simply republish whatever v & omega we receive."""

        self.pub.publish(msg)
 
    def run(self):

        rospy.spin()
 
if __name__ == '__main__':

    node = TwistRelayNode(node_name='twist_relay_node')

    node.run()
 '''
