import cv2
import numpy as np
 
class ColorRange:
    def __init__(self, low_1, high_1, low_2=None, high_2=None):
        """
        Initialize one or two HSV ranges.
        If low_2 and high_2 are provided, this object handles two ranges (e.g., red).
        """
        self.low_1 = np.array(low_1, dtype=np.uint8)
        self.high_1 = np.array(high_1, dtype=np.uint8)
        self.low_2 = np.array(low_2, dtype=np.uint8) if low_2 else None
        self.high_2 = np.array(high_2, dtype=np.uint8) if high_2 else None
 
    @staticmethod
    def fromDict(config):
        """
        Create a ColorRange object from a dictionary of thresholds.
        Works for one range (low/high) or two-part range (low_1/high_1 + low_2/high_2).
        """
        if 'low' in config and 'high' in config:
            return ColorRange(config['low'], config['high'])
        elif 'low_1' in config and 'high_1' in config and 'low_2' in config and 'high_2' in config:
            return ColorRange(config['low_1'], config['high_1'], config['low_2'], config['high_2'])
        else:
            raise ValueError("Invalid config dictionary for ColorRange")
 
    def inRange(self, hsv_image):
        """
        Apply the HSV range(s) to the input image and return a binary mask.
        Supports combining two ranges (useful for red).
        """
        mask1 = cv2.inRange(hsv_image, self.low_1, self.high_1)
        if self.low_2 is not None and self.high_2 is not None:
            mask2 = cv2.inRange(hsv_image, self.low_2, self.high_2)
            return cv2.bitwise_or(mask1, mask2)
        return mask1
if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='color_range_node')
    # keep spinning
    rospy.spin()
