#!/usr/bin/env python3

import os
import rospy
from sensor_msgs.msg import CompressedImage
import cv2  # importing OpenCV for image processing
from cv_bridge import CvBridge  # to convert between ROS image messages and OpenCV images

class CameraReaderNode:
    def __init__(self, node_name):
        # initialize the ROS node with the provided name
        rospy.init_node(node_name, anonymous=True)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']  # Must be set in environment
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        # bridge between OpenCV and ROS
        self._bridge = CvBridge()
        # construct subscriber
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)

    def callback(self, msg):
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        # display frame
        #cv2.imshow(self._window, image)
        #cv2.waitKey(1)  # Commented out to avoid GUI in headless environment

if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='camera_reader_node')
    # keep spinning
    rospy.spin()
