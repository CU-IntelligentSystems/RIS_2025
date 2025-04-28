#!/usr/bin/env python3
import rospy
import tf.transformations as tf_trans
import math
from duckietown_msgs.msg import WheelEncoderStamped # Input message type
from nav_msgs.msg import Odometry # Output message type
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import numpy as np # For covariance matrix
from threading import Lock # To protect access to tick data

class DuckieOdomConverter:
    def __init__(self):
        rospy.init_node('duckie_odom_converter', anonymous=True)

        # --- Parameters ---
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'ente/base')
        self.publish_rate = rospy.get_param('~publish_rate', 30.0)

        self.radius = rospy.get_param('/ente/kinematics_node/radius', 0.0318)
        self.baseline = rospy.get_param('/ente/kinematics_node/baseline', 0.1)
        self.ticks_per_rev = rospy.get_param('~ticks_per_revolution', 135.0)

        if self.ticks_per_rev <= 0: rospy.logerr("ticks_per_revolution must be positive."); exit()
        if self.radius <= 0: rospy.logerr("radius must be positive."); exit()
        if self.baseline <= 0: rospy.logerr("baseline must be positive."); exit()

        self.dist_per_tick = (2.0 * math.pi * self.radius) / self.ticks_per_rev

        # --- Covariance Parameters 
        self.pose_cov_x = rospy.get_param('~pose_covariance/x', 1e-4) # Using smaller test values
        self.pose_cov_y = rospy.get_param('~pose_covariance/y', 1e-4)
        self.pose_cov_yaw = rospy.get_param('~pose_covariance/yaw', 1e-5)
        self.twist_cov_vx = rospy.get_param('~twist_covariance/vx', 1e-5)
        self.twist_cov_vyaw = rospy.get_param('~twist_covariance/vyaw', 1e-5)

        # --- State Variables ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_update_time = rospy.Time.now() # Use current time for first delta_t

        self.current_ticks_left = None
        self.current_ticks_right = None
        self.current_stamp_left = None
        self.current_stamp_right = None

        self.last_calc_ticks_left = None
        self.last_calc_ticks_right = None
        self.last_calc_stamp_left = None
        self.last_calc_stamp_right = None

        self.data_lock = Lock() # Protect access when callbacks update data

        # --- Publishers ---
        self.odom_pub = rospy.Publisher('/ente/odom_converted', Odometry, queue_size=10)

        # --- Subscribers ---
        rospy.Subscriber('/ente/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_encoder_callback, queue_size=5)
        rospy.Subscriber('/ente/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_encoder_callback, queue_size=5)

        # --- Timer ---
        self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.publish_odometry)

        rospy.loginfo("Duckie Odometry Converter Initialized (v3 - Single Print).")
        rospy.loginfo(f"  Using radius={self.radius}, baseline={self.baseline}, ticks_per_rev={self.ticks_per_rev}")
        rospy.loginfo(f"  Publishing to /ente/odom_converted")

    def left_encoder_callback(self, msg):
        with self.data_lock:
            if self.last_calc_ticks_left is None:
                self.last_calc_ticks_left = msg.data
                self.last_calc_stamp_left = msg.header.stamp
            self.current_ticks_left = msg.data
            self.current_stamp_left = msg.header.stamp

    def right_encoder_callback(self, msg):
        with self.data_lock:
            if self.last_calc_ticks_right is None:
                self.last_calc_ticks_right = msg.data
                self.last_calc_stamp_right = msg.header.stamp
            self.current_ticks_right = msg.data
            self.current_stamp_right = msg.header.stamp

    def publish_odometry(self, event):
        calc_ticks_left = None
        calc_ticks_right = None
        calc_stamp_left = None
        calc_stamp_right = None

        with self.data_lock:
            if self.last_calc_ticks_left is None or self.last_calc_ticks_right is None or \
               self.current_ticks_left is None or self.current_ticks_right is None:
                return # Not initialized yet

            # Copy current state to local variables for this calculation cycle
            calc_ticks_left = self.current_ticks_left
            calc_ticks_right = self.current_ticks_right
            calc_stamp_left = self.current_stamp_left
            calc_stamp_right = self.current_stamp_right

        current_time = max(calc_stamp_left, calc_stamp_right)
        last_time = self.last_update_time

        if current_time <= last_time:
             return # Avoid zero or negative dt

        delta_t = (current_time - last_time).to_sec()

        delta_ticks_left = calc_ticks_left - self.last_calc_ticks_left
        delta_ticks_right = calc_ticks_right - self.last_calc_ticks_right

        dist_left = delta_ticks_left * self.dist_per_tick
        dist_right = delta_ticks_right * self.dist_per_tick

        d_center = (dist_left + dist_right) / 2.0
        delta_theta = (dist_right - dist_left) / self.baseline

        vx = d_center / delta_t
        vyaw = delta_theta / delta_t

        avg_theta = self.theta + delta_theta / 2.0
        delta_x = d_center * math.cos(avg_theta)
        delta_y = d_center * math.sin(avg_theta)

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        while self.theta > math.pi: self.theta -= 2.0 * math.pi
        while self.theta < -math.pi: self.theta += 2.0 * math.pi

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        quat = tf_trans.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(*quat)

        odom.pose.covariance = np.diag([
            self.pose_cov_x, self.pose_cov_y, 1e-9, 1e-9, 1e-9, self.pose_cov_yaw
        ]).flatten().tolist()

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = vyaw

        odom.twist.covariance = np.diag([
            self.twist_cov_vx, 1e-9, 1e-9, 1e-9, 1e-9, self.twist_cov_vyaw
        ]).flatten().tolist()

        # --- THE SINGLE DEBUG PRINT ---
        rospy.loginfo(f"Publishing Odom: X={odom.pose.pose.position.x:.3f} Y={odom.pose.pose.position.y:.3f} Th={self.theta:.3f} Vx={odom.twist.twist.linear.x:.3f} Vyaw={odom.twist.twist.angular.z:.3f}")
        # --- END DEBUG PRINT ---

        try:
            self.odom_pub.publish(odom)
        except Exception as e:
            rospy.logerr(f"!!! Failed to publish odom message: {e}")

        # Update state for next calculation AFTER publishing
        self.last_update_time = current_time
        self.last_calc_ticks_left = calc_ticks_left
        self.last_calc_ticks_right = calc_ticks_right
        # Update associated stamps as well
        self.last_calc_stamp_left = calc_stamp_left
        self.last_calc_stamp_right = calc_stamp_right


if __name__ == '__main__':
    try:
        converter = DuckieOdomConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
