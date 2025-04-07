#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import Imu


class IMUReaderNode(DTROS):

    def __init__(self, node_name):
        super(IMUReaderNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # get vehicle name from environment
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._imu_topic = f"/{self._vehicle_name}/imu_node/imu_data"

        # temporary data storage
        self._latest_imu = None

        # subscriber
        self.sub = rospy.Subscriber(self._imu_topic, Imu, self.imu_callback)

    def imu_callback(self, data):
        self._latest_imu = data

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self._latest_imu:
                acc = self._latest_imu.linear_acceleration
                gyro = self._latest_imu.angular_velocity
                msg = f"IMU Linear Acc [x y z]: [{acc.x:.2f}, {acc.y:.2f}, {acc.z:.2f}] | Angular Vel [x y z]: [{gyro.x:.2f}, {gyro.y:.2f}, {gyro.z:.2f}]"
                rospy.loginfo(msg)
            rate.sleep()


if __name__ == '__main__':
    node = IMUReaderNode(node_name='imu_reader_node')
    node.run()
    rospy.spin()
