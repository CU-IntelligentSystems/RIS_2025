import cv2

from color_range import ColorRange

from line_detector import LineDetector  # Your implemented class

from detections import Detections       # Your detection structure
 
# -------------------------

# 1. Define Color Ranges

# -------------------------

color_config = {

    # White range

    'low_1': [0, 0, 180],

    'high_1': [180, 40, 255],

    # Red range (split across hue wraparound)

    'low_2': [0, 70, 50],

    'high_2': [10, 255, 255],

    'low_3': [170, 70, 50],

    'high_3': [180, 255, 255]

}
 
# Create individual ColorRange objects

WHITE_RANGE = ColorRange.fromDict({

    'low': color_config['low_1'],

    'high': color_config['high_1']

})
 
RED_RANGE = ColorRange.fromDict({

    'low_1': color_config['low_2'],

    'high_1': color_config['high_2'],

    'low_2': color_config['low_3'],

    'high_2': color_config['high_3']

})
 
# -------------------------

# 2. Load Image

# -------------------------

image = self._bridge.compressed_imgmsg_to_cv2(msg)  # Replace with actual path or frame from ROS topic
 
# -------------------------

# 3. Detect Lines

# -------------------------

detector = LineDetector()

detector.setImage(image)
 
# Detect white lines

white_detections = detector.detectLines(WHITE_RANGE)

 
# Detect red lines

red_detections = detector.detectLines(RED_RANGE)

 
 
if __name__ == '__main__':

    # create the node

    node = CameraReaderNode(node_name='Line_detector_node')

    # keep spinning

    rospy.spin()
 
