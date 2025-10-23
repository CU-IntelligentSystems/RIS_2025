#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32  # Used to publish distance and detection status

import cv2
from cv_bridge import CvBridge

class CameraReaderNode(DTROS):
    def __init__(self, node_name):
        # Initialize the DTROS base class for integration with Duckietown infrastructure
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)

        # Get vehicle name from environment and build camera topic string
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"

        # Bridge to convert ROS image messages into OpenCV format
        self._bridge = CvBridge()

        # Create a named OpenCV window to visualize frames
        self._window = "camera-reader"
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)

        # Subscribe to the compressed camera feed
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)

        # Publishers for estimated object distance and detection status (used by other nodes like PID or LED control)
        self.distance_pub = rospy.Publisher('/object_distance', Float32, queue_size=10)
        self.detection_status_pub = rospy.Publisher('/object_detection_status', Float32, queue_size=10)

        # Calibration constants — adjust based on your setup!
        self.reference_object_width = 0.055  # Real width of object (e.g., duck) in meters
        self.reference_distance = 0.15       # Known distance when object appears a certain width
        self.reference_width_pixels = 20     # Width of object in pixels at reference distance

        # Scaling factor is derived for estimating distances
        self.scaling_factor = self.reference_object_width / self.reference_width_pixels

    def callback(self, msg):
        # Convert ROS compressed image message to OpenCV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)

        # Convert image to HSV color space for more robust color detection
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define HSV ranges for detecting red (split into two due to hue wrap-around)
        lower_red1 = (0, 100, 100)
        upper_red1 = (10, 255, 255)
        lower_red2 = (160, 100, 100)
        upper_red2 = (180, 255, 255)

        # Create masks for red detection and combine both
        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        # Clean up the mask to remove noise
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, None)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, None)

        # Find contours (blobs) in the red mask
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Variables for tracking the most prominent red object
        max_area = 0
        largest_contour = None
        distance = None

        # Loop through all contours to find the largest red object
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # Ignore small noise
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)  # Draw green box

                if cv2.contourArea(contour) > max_area:
                    max_area = cv2.contourArea(contour)
                    largest_contour = contour
                    distance = self.estimate_distance(w)  # Estimate distance from bounding box width

        # If a red object is detected and a distance is calculated
        if distance is not None:
            # Show distance as text on the image
            cv2.putText(image, f"Distance: {distance:.2f} m", (50, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)

            # Publish the distance for other nodes (e.g., stopping logic)
            self.distance_pub.publish(distance)

            # Publish detection status: 1 if object within 1 meter, else 0
            self.detection_status_pub.publish(1.0 if distance < 1.0 else 0.0)
        else:
            # No object found — publish 0
            self.detection_status_pub.publish(0.0)

        # Show the image with boxes and text overlay
        cv2.imshow(self._window, image)
        cv2.waitKey(1)

    def estimate_distance(self, object_width):
        """
        Estimate distance to the object using known physical width and apparent width in pixels.
        :param object_width: Width of object in pixels
        :return: Estimated distance in meters
        """
        if object_width == 0:
            return float('inf')  # Prevent division by zero

        # Estimate the distance proportionally based on reference calibration
        distance = self.reference_distance * (self.reference_width_pixels / object_width)
        return distance

if __name__ == '__main__':
    # Create and run the camera node
    node = CameraReaderNode(node_name='camera_reader_node')
    rospy.spin()
