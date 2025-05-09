from std_msgs.msg import Float32, Bool
 
offset_pub = rospy.Publisher('/lane_offset', Float32, queue_size=1)
redline_pub = rospy.Publisher('/red_line_detected', Bool, queue_size=1)
 
offset_pub.publish(offset)
redline_pub.publish(red_flag)
 
if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='Publish_info_node')
    # keep spinning
    rospy.spin()
