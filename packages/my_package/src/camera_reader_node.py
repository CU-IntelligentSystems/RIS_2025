#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage

import cv2
from cv_bridge import CvBridge

class CameraReaderNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        # bridge between OpenCV and ROS
        self._bridge = CvBridge()
        # create window
        self._window = "camera-reader"
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        # construct subscriber
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)

    def callback(self, msg):
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        
        # Convert the image to HSV (Hue, Saturation, Value) color space
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define the range for yellow color in HSV space
        # These values can be adjusted depending on your yellow's exact shade
        lower_yellow = (20, 100, 100)  # Lower bound for yellow
        upper_yellow = (40, 255, 255)  # Upper bound for yellow

        # Create a binary mask where yellow areas are white, others are black
        yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

        # Perform some cleaning of the mask (optional)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, None)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, None)

        # Find contours of the yellow areas in the mask
        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw bounding boxes around the detected yellow areas
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # Filter small contours
                # Get the bounding box for the contour
                x, y, w, h = cv2.boundingRect(contour)
                # Draw a rectangle around the detected yellow area
                cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)

        # Display the frame with bounding boxes around yellow areas
        cv2.imshow(self._window, image)
        cv2.waitKey(1)

if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='camera_reader_node')
    # keep spinning
    rospy.spin()
