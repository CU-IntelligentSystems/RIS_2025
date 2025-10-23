#!/usr/bin/env python3
 
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped
 
class TwistToWheelsNode(DTROS):
    def __init__(self, node_name):
        super().__init__(node_name=node_name, node_type=NodeType.CONTROL)
        self.L = rospy.get_param("~wheelbase", 0.1)  # Distance between wheels
        self.sub = rospy.Subscriber("car_cmd", Twist2DStamped, self.callback)
        self.pub = rospy.Publisher("wheels_cmd", WheelsCmdStamped, queue_size=1)
        rospy.loginfo("TwistToWheelsNode initialized")
 
    def callback(self, msg):
        v = msg.v
        omega = msg.omega
        v_left = v - 0.5 * self.L * omega
        v_right = v + 0.5 * self.L * omega
        wheel_msg = WheelsCmdStamped()
        wheel_msg.vel_left = v_left
        wheel_msg.vel_right = v_right
        self.pub.publish(wheel_msg)
 
    def run(self):
        rospy.spin()
 
if __name__ == '__main__':
    node = TwistToWheelsNode(node_name='twist_to_wheels_node')
    node.run()
