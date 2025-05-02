#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32  # Import for distance and detection status publishing

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

        # Publisher for object distance and detection status
        self.distance_pub = rospy.Publisher('/object_distance', Float32, queue_size=10)
        self.detection_status_pub = rospy.Publisher('/object_detection_status', Float32, queue_size=10)

        # Define a reference object size and distance for scaling (adjust these values)
        self.reference_object_width = 0.01  # Real width of the duck in meters (5.5 cm)
        self.reference_distance = 0.15  # Real distance of the duck in meters (15 cm)
        self.reference_width_pixels = 20  # Bounding box width in pixels when the object is at 15 cm

        # Calculate the scaling factor based on the reference data
        self.scaling_factor = self.reference_object_width / self.reference_width_pixels

    def callback(self, msg):
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)

        # Convert the image to HSV (Hue, Saturation, Value) color space
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define lower and upper bounds for red in HSV
        lower_red1 = (0, 100, 100)
        upper_red1 = (10, 255, 255)

        lower_red2 = (160, 100, 100)
        upper_red2 = (180, 255, 255)

        # Create two masks and combine them
        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

        # Combine the masks
        red_mask = cv2.bitwise_or(mask1, mask2)

        # Perform some cleaning of the mask (optional)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, None)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, None)

        # Find contours of the detected red areas in the mask
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Initialize variables for tracking the largest contour (red object)
        max_area = 0
        largest_contour = None
        distance = None  # Variable to store the distance to the detected object

        # Draw bounding boxes around the detected areas and calculate their area
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # Filter small contours
                # Get the bounding box for the contour
                x, y, w, h = cv2.boundingRect(contour)
                # Draw a rectangle around the detected red area
                cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)

                # Keep track of the largest bounding box (assumed to be the red object)
                if cv2.contourArea(contour) > max_area:
                    max_area = cv2.contourArea(contour)
                    largest_contour = contour
                    # Approximate the distance based on the size of the bounding box
                    distance = self.estimate_distance(w)  # Use width of bounding box to estimate distance

        # If an object (red) is detected, show the estimated distance
        if distance is not None:
            # Display the distance on the image (in meters)
            cv2.putText(image, f"Distance: {distance:.2f} m", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)
            
            # Publish the distance to the /object_distance topic
            self.distance_pub.publish(distance)

            # Publish detection status (1 for detected, 0 for not detected)
            if distance < 1.0:  # Object is detected and within 1 meter range
                self.detection_status_pub.publish(1.0)
            else:
                self.detection_status_pub.publish(0.0)
        else:
            # If no object is detected, send a detection status of 0.0
            self.detection_status_pub.publish(0.0)

        # Display the frame with bounding boxes around detected areas and distance
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
