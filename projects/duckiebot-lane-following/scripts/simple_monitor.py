#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
import time

class SimpleMonitor(Node):
    def __init__(self):
        super().__init__('simple_monitor')
        
        # Subscribers
        self.lane_sub = self.create_subscription(
            Bool, '/lane_detected', self.lane_callback, 10)
        self.obstacle_sub = self.create_subscription(
            Bool, '/obstacle_detected', self.obstacle_callback, 10)
        self.state_sub = self.create_subscription(
            String, '/robot_state', self.state_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/duckiebot/cmd_vel', self.cmd_vel_callback, 10)
        
        # Status tracking
        self.lane_detected = False
        self.obstacle_detected = False
        self.robot_state = "UNKNOWN"
        self.current_speed = 0.0
        self.current_angular = 0.0
        
        # Statistics
        self.lane_detections = 0
        self.obstacle_detections = 0
        self.state_changes = 0
        self.start_time = time.time()
        
        # Create timer for status reports
        self.timer = self.create_timer(8.0, self.report_status)
        
        self.get_logger().info("📊 Simple Monitor started - Camera-based obstacle avoidance")
        self.display_welcome_info()
        
    def display_welcome_info(self):
        """Display system information"""
        print("\n" + "="*60)
        print("📷 CAMERA-ONLY OBSTACLE AVOIDANCE SYSTEM")
        print("="*60)
        print("🎯 FEATURES:")
        print("   • Lane detection using yellow line recognition")
        print("   • Obstacle detection using color-based computer vision")
        print("   • No additional sensors required - camera only!")
        print("   • Real-time visual debugging")
        print("\n🤖 HOW IT WORKS:")
        print("   • Detects yellow lanes in bottom area of camera view")
        print("   • Detects colorful obstacles in middle area of camera view")
        print("   • Estimates obstacle distance based on size and position")
        print("   • Avoids obstacles by temporarily leaving the lane")
        print("   • Returns to lane following after obstacle clearance")
        print("\n📺 TOPICS TO WATCH:")
        print("   • /lane_detected - Lane detection status")
        print("   • /obstacle_detected - Obstacle detection status")
        print("   • /robot_state - Current behavior state")
        print("   • /lane_debug - Visual debug image")
        print("\n🔧 USEFUL COMMANDS:")
        print("   View debug image: ros2 run image_view image_view image:=/lane_debug")
        print("   Monitor states: ros2 topic echo /robot_state")
        print("   Check detection: ros2 topic echo /obstacle_detected")
        print("="*60 + "\n")
        
    def lane_callback(self, msg):
        if msg.data and not self.lane_detected:
            self.lane_detections += 1
        self.lane_detected = msg.data
    
    def obstacle_callback(self, msg):
        if msg.data and not self.obstacle_detected:
            self.obstacle_detections += 1
        self.obstacle_detected = msg.data
    
    def state_callback(self, msg):
        if msg.data != self.robot_state:
            self.state_changes += 1
            old_state = self.robot_state
            self.robot_state = msg.data
            self.get_logger().info(f"🔄 State: {old_state} → {self.robot_state}")
    
    def cmd_vel_callback(self, msg):
        self.current_speed = msg.linear.x
        self.current_angular = msg.angular.z
    
    def report_status(self):
        """Periodic status report"""
        runtime = time.time() - self.start_time
        
        self.get_logger().info("📊 === CAMERA-BASED SYSTEM STATUS ===")
        self.get_logger().info(f"   Runtime: {runtime:.1f}s")
        self.get_logger().info(f"   Current State: {self.robot_state}")
        self.get_logger().info(f"   Lane: {'DETECTED' if self.lane_detected else 'LOST'}")
        self.get_logger().info(f"   Obstacle: {'DETECTED' if self.obstacle_detected else 'CLEAR'}")
        self.get_logger().info(f"   Speed: {self.current_speed:.2f} m/s")
        self.get_logger().info(f"   Angular: {self.current_angular:.2f} rad/s")
        self.get_logger().info(f"   Statistics:")
        self.get_logger().info(f"     - Lane detections: {self.lane_detections}")
        self.get_logger().info(f"     - Obstacle detections: {self.obstacle_detections}")
        self.get_logger().info(f"     - State changes: {self.state_changes}")
        
        # Performance tips
        if self.obstacle_detections == 0 and runtime > 30:
            self.get_logger().info("💡 No obstacles detected yet - check if obstacles are colorful enough")
        elif self.lane_detections == 0 and runtime > 10:
            self.get_logger().info("💡 No lane detected - check yellow line visibility")
        
        self.get_logger().info("====================================")

def main(args=None):
    rclpy.init(args=args)
    
    monitor = SimpleMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()