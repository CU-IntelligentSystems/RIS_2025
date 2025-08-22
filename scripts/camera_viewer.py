#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        
        self.declare_parameter('camera_topic', '/duckiebot/camera/image_raw')
        camera_topic = self.get_parameter('camera_topic').value
        
        self.bridge = CvBridge()
        self.frame_count = 0
        self.last_time = time.time()
        self.fps = 0.0
        
        self.image_sub = self.create_subscription(
            Image, camera_topic, self.image_callback, 10)
        
        cv2.namedWindow("ROS 2 Duckiebot Camera", cv2.WINDOW_AUTOSIZE)
        self.get_logger().info("📺 ROS 2 Camera viewer started!")
        self.get_logger().info("Press 'q' to quit")
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Calculate FPS
            self.frame_count += 1
            current_time = time.time()
            if current_time - self.last_time > 1.0:
                self.fps = self.frame_count / (current_time - self.last_time)
                self.frame_count = 0
                self.last_time = current_time
            
            # Add ROS 2 overlay
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(cv_image, f"ROS 2 Foxy - FPS: {self.fps:.1f}", 
                       (10, 30), font, 0.7, (0, 255, 0), 2)
            
            h, w = cv_image.shape[:2]
            cv2.putText(cv_image, f"Size: {w}x{h}", 
                       (10, 60), font, 0.5, (0, 255, 0), 1)
            
            cv2.imshow("ROS 2 Duckiebot Camera", cv_image)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info("Shutting down camera viewer")
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    viewer = CameraViewer()
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
