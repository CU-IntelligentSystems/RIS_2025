#!/usr/bin/env python3

import rospy
import os
import cv2  # importing OpenCV for image processing - detecting the red line.
import numpy as np  # for numerical operations, particularly handling image data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from cv_bridge import CvBridge  # to convert between ROS image messages and OpenCV images

class RedLineDetector:
    def __init__(self, node_name):  # constructor
        # Initialize the ROS node with the provided name
        rospy.init_node(node_name, anonymous=True)
        # Static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']  # Must be set in environment
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        # Bridge between OpenCV and ROS
        self._bridge = CvBridge()
        # Creating window
        # self._window = "camera-reader"
        # cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        # Constructing subscriber and publisher
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.image_callback)
        self.red_line_pub = rospy.Publisher(f"/{self._vehicle_name}/red_line", Bool, queue_size=1)
        # State variable
        self.red_line_detected = False  # initializing a boolean state variable to track whether a red line detected. This prevents repeated publishing of the same detection event

    def image_callback(self, msg):
        try:
            # Convert image
            cv_image = self._bridge.compressed_imgmsg_to_cv2(msg)
            # Display frame
            # cv2.imshow(self._window, cv_image)
            # cv2.waitKey(1)
            # Red line detection
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)  # converting the image from BGR to HSV
            lower_red = np.array([0, 100, 100])  # defining a lower bound for the red HSV
            upper_red = np.array([10, 255, 255])  # Defining the upper bound for red in HSV
            mask = cv2.inRange(hsv, lower_red, upper_red)  # creating a binary mask where pixels within the red range are white (255), and others are black (0).
            # Check for significant red area
            if np.sum(mask) > 500000:  # Threshold may need tuning
                if not self.red_line_detected:  # avoiding spamming or multiple publishing
                    rospy.loginfo("Red line detected!")
                    self.red_line_pub.publish(Bool(data=True))
                    self.red_line_detected = True
            else:
                self.red_line_detected = False
        except Exception as e:  # catching errors during image processing such as invalid image data
            rospy.logerr(f"Image processing error: {e}")

if __name__ == '__main__':
    # Create the node
    node = RedLineDetector(node_name='red_line_detector')
    # Keep spinning
    rospy.spin()
