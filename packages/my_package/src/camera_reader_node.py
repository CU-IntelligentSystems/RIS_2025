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

        # Define a reference object size and distance for scaling (adjust these values)
        self.reference_object_width = 0.055  # Real width of the duck in meters (5.5 cm)
        self.reference_distance = 0.15  # Real distance of the duck in meters (15 cm)
        self.reference_width_pixels = 66  # Bounding box width in pixels when the object is at 15 cm

        # Calculate the scaling factor based on the reference data
        self.scaling_factor = self.reference_object_width / self.reference_width_pixels

    def callback(self, msg):
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        
        # Convert the image to HSV (Hue, Saturation, Value) color space
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define the range for yellow color in HSV space
        lower_yellow = (20, 100, 100)  # Lower bound for yellow
        upper_yellow = (40, 255, 255)  # Upper bound for yellow

        # Create a binary mask where yellow areas are white, others are black
        yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

        # Perform some cleaning of the mask (optional)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, None)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, None)

        # Find contours of the yellow areas in the mask
        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Initialize variables for tracking the largest contour (yellow duck)
        max_area = 0
        largest_contour = None
        distance = None  # Variable to store the distance to the duck

        # Draw bounding boxes around the detected yellow areas and calculate their area
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # Filter small contours
                # Get the bounding box for the contour
                x, y, w, h = cv2.boundingRect(contour)
                # Draw a rectangle around the detected yellow area
                cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)

                # Print the width of the bounding box (in pixels)
                print(f"Bounding Box Width: {w} pixels")

                # Keep track of the largest bounding box (assumed to be the duck)
                if cv2.contourArea(contour) > max_area:
                    max_area = cv2.contourArea(contour)
                    largest_contour = contour
                    # Approximate the distance based on the size of the bounding box
                    distance = self.estimate_distance(w)  # Use width of bounding box to estimate distance

        # If a yellow object (duck) is detected, show the estimated distance
        if distance is not None:
            # Display the distance on the image (in meters)
            cv2.putText(image, f"Distance: {distance:.2f} m", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)

        # Display the frame with bounding boxes around yellow areas and distance
        cv2.imshow(self._window, image)
        cv2.waitKey(1)

    def estimate_distance(self, object_width):
        """
        Estimate the distance to the object based on the width of its bounding box.
        Uses a scaling factor derived from the reference data.

        :param object_width: The width of the detected object in pixels
        :return: Estimated distance to the object in meters
        """
        # Estimate the distance to the object using the scaling factor
        if object_width == 0:  # To avoid division by zero
            return float('inf')  # Return a very large number if the object width is zero (no object detected)

        distance = self.reference_distance * (self.reference_width_pixels / object_width)

        return distance

if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='camera_reader_node')
    # keep spinning
    rospy.spin()
