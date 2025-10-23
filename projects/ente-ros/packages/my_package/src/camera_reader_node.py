#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge

class CameraReaderNode(DTROS):
    def __init__(self, node_name):
        # Initialize the DTROS parent class with a visualization node type
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        self._vehicle_name = os.environ.get('VEHICLE_NAME', 'default_vehicle')
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        # Initialize CvBridge for converting ROS images to OpenCV images
        self._bridge = CvBridge()
        # Create an OpenCV window to display the camera feed
        self._window = "camera-reader"
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        # Set up the subscriber to the camera topic
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)

    def callback(self, msg):
        try:
            # Convert the compressed image message to an OpenCV image
            image = self._bridge.compressed_imgmsg_to_cv2(msg)
            # Display the image in the window
            cv2.imshow(self._window, image)
            # Wait a short period for the window to update
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")

if __name__ == '__main__':
    # Instantiate the node with the name 'camera_reader_node'
    node = CameraReaderNode(node_name='camera_reader_node')
    # Keep the node running
    rospy.spin()
