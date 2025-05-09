mport cv2
from color_range import ColorRange
from line_detector import LineDetector
 
# 1. Define color ranges
color_config = {
    'low_1': [0, 0, 180], 'high_1': [180, 40, 255],  # White
    'low_2': [0, 70, 50], 'high_2': [10, 255, 255],  # Red range 1
    'low_3': [170, 70, 50], 'high_3': [180, 255, 255]  # Red range 2
}
 
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
 
# 2. Load image
image = cv2.imread("duckietown_sample.jpg")
 
# 3. Detect lines
detector = LineDetector()
detector.setImage(image)
 
white_detections = detector.detectLines(WHITE_RANGE)
red_detections = detector.detectLines(RED_RANGE)
 
# 4. Set red stop flag
red_flag = len(red_detections.lines) > 0
 
 
if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='flag_readline_node')
    # keep spinning
    rospy.spin()
