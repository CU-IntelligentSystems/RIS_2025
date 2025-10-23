#!/usr/bin/env python3
"""
One-node white-line segmentation and line detection for Duckiebot (ROS Melodic, DTROS)
Combines white-pixel segmentation, line fitting, centroid & orientation extraction, and control.
"""
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32
from duckietown_msgs.msg import Twist2DStamped
from duckietown.dtros import DTROS, NodeType, TopicType

class WhiteLineDetectorFollowerNode(DTROS):
    def __init__(self, node_name):
        super(WhiteLineDetectorFollowerNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.CONTROL
        )
        # Initialization
        self.bridge = CvBridge()

        # Parameters
        hsv_lo = rospy.get_param('~hsv_lower', [0, 0, 230])
        hsv_hi = rospy.get_param('~hsv_upper', [180, 50, 255])
        self.hsv_lower = np.array(hsv_lo, dtype=np.uint8)
        self.hsv_upper = np.array(hsv_hi, dtype=np.uint8)
        self.roi_frac = rospy.get_param('~roi_fraction', 0.5)
        self.top_cutoff = rospy.get_param('~top_cutoff', 0)

        # Hough parameters
        self.hough_rho = rospy.get_param('~hough_rho', 1)
        self.hough_theta = rospy.get_param('~hough_theta', np.pi/360)
        self.hough_threshold = rospy.get_param('~hough_threshold', 80)
        self.hough_min_line = rospy.get_param('~hough_min_line', 60)
        self.hough_max_gap = rospy.get_param('~hough_max_gap', 10)

        # Control gains
        self.Kp_pos = rospy.get_param('~Kp_pos', 0.005)
        self.Kp_ang = rospy.get_param('~Kp_ang', 1.0)
        self.v_straight = rospy.get_param('~v_straight', 0.2)
        self.v_turn = rospy.get_param('~v_turn', 0.1)
        self.err_thresh = rospy.get_param('~err_thresh', 20.0)

        self.debug = rospy.get_param('~debug', False)

        # Publishers
        self.pub_error = rospy.Publisher('~line_error', Float32, queue_size=1, dt_topic_type=TopicType.DEBUG)
        self.pub_angle = rospy.Publisher('~line_angle', Float32, queue_size=1, dt_topic_type=TopicType.DEBUG)
        self.pub_cmd = rospy.Publisher('car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL)
        
        rospy.on_shutdown(self.on_shutdown)
        if self.debug:
            self.pub_mask = rospy.Publisher('~debug/mask', Image, queue_size=1, dt_topic_type=TopicType.DEBUG)
            self.pub_hough = rospy.Publisher('~debug/hough', Image, queue_size=1, dt_topic_type=TopicType.DEBUG)

        # Subscriber
        self.sub_image = rospy.Subscriber('~image/compressed', CompressedImage,
                                          self.image_cb, buff_size=2**24, queue_size=1)

        rospy.loginfo('WhiteLineDetectorFollowerNode initialized')

    def image_cb(self, msg):
        # 1. Decode image
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return
        if self.debug:
            rospy.loginfo('🖼️ image_cb() received frame')

        # 2. Resize & crop ROI
        h, w = frame.shape[:2]
        new_w = rospy.get_param('~img_width', w)
        new_h = rospy.get_param('~img_height', h)
        frame = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
        y0 = int((1.0 - self.roi_frac) * new_h) + self.top_cutoff
        roi = frame[y0:new_h, :]

        # 3. White segmentation
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))

        # 4. Centroid error
        M = cv2.moments(mask)
        err = 0.0
        if M['m00'] > 0:
            cx = M['m10'] / M['m00']
            err = cx - (new_w / 2.0)
        self.pub_error.publish(err)

        # 5. Hough line detection for orientation
        angle = 0.0
        lines = cv2.HoughLinesP(mask,
                                self.hough_rho,
                                self.hough_theta,
                                self.hough_threshold,
                                minLineLength=self.hough_min_line,
                                maxLineGap=self.hough_max_gap)
        if lines is not None:
            angles = [np.arctan2((y1 - y2), (x2 - x1)) for x1, y1, x2, y2 in lines[:,0]]
            angle = float(np.mean(angles))
        self.pub_angle.publish(angle)

        # 6. Control law (combine position & heading)
        omega = - (self.Kp_pos * err + self.Kp_ang * angle)
        v = self.v_straight if abs(err) < self.err_thresh else self.v_turn

        # Debug: log control values
        if self.debug:
            rospy.loginfo('err=%.1f px, angle=%.2f rad, v=%.2f, ω=%.2f', err, angle, v, omega)

        cmd = Twist2DStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.v = v
        cmd.omega = omega
        self.pub_cmd.publish(cmd)
        


        # 7. Debug visuals
        if self.debug:
            mask_img = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
            mask_img.header = msg.header
            self.pub_mask.publish(mask_img)

            vis = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            if lines is not None:
                for x1, y1, x2, y2 in lines[:,0]:
                    cv2.line(vis, (int(x1),int(y1)), (int(x2),int(y2)), (0,255,0), 2)
            hough_msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
            hough_msg.header = msg.header
            self.pub_hough.publish(hough_msg)
        if self.debug:
            mask_img = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
            mask_img.header = msg.header
            self.pub_mask.publish(mask_img)

            vis = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            if lines is not None:
                for x1, y1, x2, y2 in lines[:,0]:
                    cv2.line(vis, (int(x1),int(y1)), (int(x2),int(y2)), (0,255,0), 2)
            hough_msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
            hough_msg.header = msg.header
            self.pub_hough.publish(hough_msg)
            
	
    def on_shutdown(self):
        rospy.loginfo("Shutting down, sending stop command...")
        stop_cmd = Twist2DStamped()
        stop_cmd.v = 0.0
        stop_cmd.omega = 0.0
        stop_cmd.header.stamp = rospy.Time.now()
        self.pub_cmd.publish(stop_cmd)
 
    def run(self):
        rospy.spin()
        

if __name__ == '__main__':
    node = WhiteLineDetectorFollowerNode(node_name='white_line_detector_follower')
    node.run()

