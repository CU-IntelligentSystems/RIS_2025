def calculate_lane_offset(mask):
    """
    Calculates horizontal offset of white/red region from image center.
    Returns positive if to the right, negative if to the left.
    """
    height, width = mask.shape
    roi = mask[int(height * 0.6):, :]  # Use bottom 40% of image
    M = cv2.moments(roi)
 
    if M["m00"] > 0:
        cx = int(M["m10"] / M["m00"])  # Centroid x
        offset = cx - width // 2       # Positive = right, Negative = left
    else:
        offset = 0  # No lane detected
 
    return offset
if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='Detect_offset_node')
    # keep spinning
    rospy.spin()
