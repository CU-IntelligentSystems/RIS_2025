#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
import numpy as np # Used for creating diagonal matrices easily

class ImuCovarianceFixer:
    def __init__(self):
        rospy.init_node('imu_covariance_fixer', anonymous=True)

        # --- Parameters ---
        # Input topic (original IMU data)
        self.input_topic = rospy.get_param('~input_topic', '/ente/imu_node/data')
        # Output topic (IMU data with fixed covariances)
        self.output_topic = rospy.get_param('~output_topic', '/ente/imu_node/data_fixed')

        # Fixed covariance values (Variances on the diagonal) - TUNE THESE!
        # Angular velocity (roll, pitch, yaw rates) variance
        cov_vv = rospy.get_param('~angular_velocity_variance', 0.001) # e.g., (0.03 rad/s)^2
        # Linear acceleration (x, y, z) variance
        cov_aa = rospy.get_param('~linear_acceleration_variance', 0.005) # e.g., (0.07 m/s^2)^2

        # --- Create Covariance Matrices (Diagonal) ---
        # Angular Velocity Covariance (Order: Vroll, VPitch, VYaw)
        self.ang_vel_cov_matrix = np.diag([cov_vv, cov_vv, cov_vv]).flatten().tolist()
        # Linear Acceleration Covariance (Order: Ax, Ay, Az)
        self.lin_acc_cov_matrix = np.diag([cov_aa, cov_aa, cov_aa]).flatten().tolist()

        # --- Publisher ---
        self.imu_pub = rospy.Publisher(self.output_topic, Imu, queue_size=10)

        # --- Subscriber ---
        rospy.Subscriber(self.input_topic, Imu, self.imu_callback, queue_size=5)

        rospy.loginfo("IMU Covariance Fixer Initialized.")
        rospy.loginfo(f"  Subscribing to: {self.input_topic}")
        rospy.loginfo(f"  Publishing to: {self.output_topic}")
        rospy.loginfo(f"  Setting angular_velocity_covariance diagonal to: {cov_vv}")
        rospy.loginfo(f"  Setting linear_acceleration_covariance diagonal to: {cov_aa}")


    def imu_callback(self, msg_in):
        # Create a copy of the message
        msg_out = msg_in

        # Overwrite the covariance matrices with fixed positive values
        # Important: Keep orientation covariance as is (likely invalid with -1 flag)
        msg_out.angular_velocity_covariance = self.ang_vel_cov_matrix
        msg_out.linear_acceleration_covariance = self.lin_acc_cov_matrix

        # Publish the modified message
        self.imu_pub.publish(msg_out)


if __name__ == '__main__':
    try:
        fixer = ImuCovarianceFixer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass