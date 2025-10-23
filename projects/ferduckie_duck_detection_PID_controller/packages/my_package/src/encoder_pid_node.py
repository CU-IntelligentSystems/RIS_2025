#!/usr/bin/env python3
'''
import os
import rospy
import time
import matplotlib.pyplot as plt
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped


class EncoderPIDNode(DTROS):
    def __init__(self, node_name):
        super(EncoderPIDNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        self._left_encoder_topic = f"/{vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{vehicle_name}/right_wheel_encoder_node/tick"

        self._ticks_left = None
        self._ticks_right = None
        self._last_tick_left = None
        self._last_tick_right = None

        # PID gains
        self.Kp = 0.005
        self.Ki = 0.0001    # 0.0001  
        self.Kd = 0.001

        self.integral = 0
        self.last_error = 0

        self.base_speed = 0.2
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)

        self.reset_encoders = False

        # self.time_log = []
        # self.left_cmd_log = []
        # self.right_cmd_log = []


    def callback_left(self, data):
        if self.reset_encoders:
            self._ticks_left = 0
            self._last_tick_left = 0
            self.reset_encoders = False
        else:
            self._ticks_left = data.data
            if self._last_tick_left is None:
                self._last_tick_left = self._ticks_left

    def callback_right(self, data):
        if self.reset_encoders:
            self._ticks_right = 0
            self._last_tick_right = 0
            self.reset_encoders = False
        else:
            self._ticks_right = data.data
            if self._last_tick_right is None:
                self._last_tick_right = self._ticks_right

    def reset_encoder_ticks(self):
        self.reset_encoders = True
        rospy.loginfo("Encoder ticks reset.")

    def run(self):
        rospy.loginfo("Running straight-line PID controller for 5 seconds...")
        rate = rospy.Rate(10)  # 10 Hz
        start_time = time.time()

        while not rospy.is_shutdown() and time.time() - start_time < 10:
            if self._ticks_left is not None and self._ticks_right is not None:
                error = self._ticks_left - self._ticks_right

                # PID computations
                self.integral += error
                derivative = error - self.last_error

                correction = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

                left_cmd = self.base_speed - correction
                right_cmd = self.base_speed + correction

                left_cmd = max(min(left_cmd, 1.0), -1.0)
                right_cmd = max(min(right_cmd, 1.0), -1.0)

                self._publisher.publish(WheelsCmdStamped(vel_left=left_cmd, vel_right=right_cmd))

                rospy.loginfo(f"[ENCODERS] Left: {self._ticks_left}, Right: {self._ticks_right}")
                rospy.loginfo(f"[CMD] L: {left_cmd:.3f}, R: {right_cmd:.3f} | Error: {error} | Correction: {correction:.4f}")

                self.last_error = error

                # current_time = time.time() - start_time
                # self.time_log.append(current_time)
                # self.left_cmd_log.append(left_cmd)
                # self.right_cmd_log.append(right_cmd)

            else:
                rospy.loginfo("Waiting for encoder data...")

            rate.sleep()

        self._publisher.publish(WheelsCmdStamped(vel_left=0.0, vel_right=0.0))
        rospy.loginfo("PID controller stopped after 5 seconds.")

        # # Plot motor commands over time
        # plt.figure()
        # plt.plot(self.time_log, self.left_cmd_log, label="Left Motor")
        # plt.plot(self.time_log, self.right_cmd_log, label="Right Motor")
        # plt.xlabel("Time (s)")
        # plt.ylabel("Motor Command")
        # plt.title("Motor Speed Commands Over Time")
        # plt.legend()
        # plt.grid(True)
        # plt.show()
        # rospy.loginfo("PID controller stopped after 5 seconds.")



if __name__ == '__main__':
    node = EncoderPIDNode(node_name='encoder_pid_node')
    node.run()










'''







import os
import rospy
import time
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped

class EncoderPIDNode(DTROS):
    def __init__(self, node_name):
        super(EncoderPIDNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        self._left_encoder_topic = f"/{vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{vehicle_name}/right_wheel_encoder_node/tick"

        self._ticks_left = None
        self._ticks_right = None
        self._last_tick_left = None
        self._last_tick_right = None

        self.Kp = 0.005
        self.base_speed = 0.2

        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)

        # Reset flag to indicate if the encoder values need to be reset
        self.reset_encoders = False

    def callback_left(self, data):
        if self.reset_encoders:
            self._ticks_left = 0  # Reset the left encoder ticks
            self._last_tick_left = 0  # Reset the last tick to zero
            self.reset_encoders = False  # Reset the flag
        else:
            self._ticks_left = data.data
            if self._last_tick_left is None:
                self._last_tick_left = self._ticks_left

    def callback_right(self, data):
        if self.reset_encoders:
            self._ticks_right = 0  # Reset the right encoder ticks
            self._last_tick_right = 0  # Reset the last tick to zero
            self.reset_encoders = False  # Reset the flag
        else:
            self._ticks_right = data.data
            if self._last_tick_right is None:
                self._last_tick_right = self._ticks_right

    def reset_encoder_ticks(self):
        self.reset_encoders = True
        rospy.loginfo("Encoder ticks reset.")

    def run(self):
        rospy.loginfo("Running straight-line P controller for 5 seconds...")
        rate = rospy.Rate(10)  # 10 Hz
        start_time = time.time()
        # node.reset_encoder_ticks()
        # rospy.loginfo(f"[ENCODERS] Left: {self._ticks_left}, Right: {self._ticks_right}")

        while not rospy.is_shutdown() and time.time() - start_time < 10:
            if self._ticks_left is not None and self._ticks_right is not None:
        
                error = self._ticks_left  - self._ticks_right
                correction = self.Kp * error

                left_cmd = self.base_speed - correction
                right_cmd = self.base_speed + correction

                # Clamp values
                left_cmd = max(min(left_cmd, 1.0), -1.0)
                right_cmd = max(min(right_cmd, 1.0), -1.0)

                self._publisher.publish(WheelsCmdStamped(vel_left=left_cmd, vel_right=right_cmd))

                rospy.loginfo(f"[ENCODERS] Left: {self._ticks_left}, Right: {self._ticks_right}")
                rospy.loginfo(f"[CMD] L: {left_cmd:.3f}, R: {right_cmd:.3f} | Error: {error} | Correction: {correction:.4f}")

        #         self._last_tick_left = self._ticks_left
        #         self._last_tick_right = self._ticks_right
            else:
                rospy.loginfo("Waiting for encoder data...")

            rate.sleep()

        # Stop after 5 seconds
        self._publisher.publish(WheelsCmdStamped(vel_left=0.0, vel_right=0.0))
        rospy.loginfo("P controller stopped after 5 seconds.")

if __name__ == '__main__':
    node = EncoderPIDNode(node_name='encoder_pid_node')
    node.run()















'''

#!/usr/bin/env python3

import os
import rospy
import time
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped

class EncoderPIDNode(DTROS):
    def __init__(self, node_name):
        super(EncoderPIDNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        self._left_encoder_topic = f"/{vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{vehicle_name}/right_wheel_encoder_node/tick"

        self._ticks_left = None
        self._ticks_right = None
        self._last_tick_left = None
        self._last_tick_right = None

        self.Kp = 0.01  # Adjust this value based on how much correction you want
        self.base_speed = 0.0

        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)

    def callback_left(self, data):
        self._ticks_left = data.data
        if self._last_tick_left is None:
            self._last_tick_left = self._ticks_left

    def callback_right(self, data):
        self._ticks_right = data.data
        if self._last_tick_right is None:
            self._last_tick_right = self._ticks_right

    def run(self):
        rospy.loginfo("Running straight-line P controller...")
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            if self._ticks_left is not None and self._ticks_right is not None:
                delta_left = self._ticks_left - self._last_tick_left
                delta_right = self._ticks_right - self._last_tick_right

                error = delta_left - delta_right
                correction = self.Kp * error

                left_cmd = self.base_speed - correction
                right_cmd = self.base_speed + correction

                # Clamp values
                left_cmd = max(min(left_cmd, 1.0), -1.0)
                right_cmd = max(min(right_cmd, 1.0), -1.0)

                self._publisher.publish(WheelsCmdStamped(vel_left=left_cmd, vel_right=right_cmd))

                rospy.loginfo(f"[ENCODERS] Left: {self._ticks_left}, Right: {self._ticks_right}")
                rospy.loginfo(f"[CMD] L: {left_cmd:.3f}, R: {right_cmd:.3f} | Error: {error} | Correction: {correction:.4f}")

                self._last_tick_left = self._ticks_left
                self._last_tick_right = self._ticks_right
            else:
                rospy.loginfo("Waiting for encoder data...")

            rate.sleep()

        # Stop at the end
        self._publisher.publish(WheelsCmdStamped(vel_left=0.0, vel_right=0.0))
        rospy.loginfo("P controller stopped.")

if __name__ == '__main__':
    node = EncoderPIDNode(node_name='encoder_pid_node')
    node.run()




'''





























'''
#!/usr/bin/env python3

import os
import rospy
import time
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.prev_error = 0.0
        self.integral = 0.0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

class EncoderPIDNode(DTROS):
    def __init__(self, node_name):
        super(EncoderPIDNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        # static parameters
        self._left_encoder_topic = f"/{vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{vehicle_name}/right_wheel_encoder_node/tick"

        # encoder state
        self._last_tick_left = None
        self._last_tick_right = None
        self._ticks_left = None
        self._ticks_right = None
        self._last_time = time.time()

        # PID controllers
        self.left_pid = PIDController(Kp=0.5, Ki=0.01, Kd=0.01)
        self.right_pid = PIDController(Kp=0.5, Ki=0.01, Kd=0.01)

        # desired speed in ticks/sec
        self.target_ticks_per_sec = 15.0

        # subscribers
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)

    def callback_left(self, data):
        self._ticks_left = data.data
        if self._last_tick_left is None:
            self._last_tick_left = self._ticks_left

    def callback_right(self, data):
        self._ticks_right = data.data
        if self._last_tick_right is None:
            self._last_tick_right = self._ticks_right

    def run(self):
        rospy.loginfo("Starting PID control loop...")
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            current_time = time.time()
            dt = current_time - self._last_time
            self._last_time = current_time

            if self._ticks_left is not None and self._ticks_right is not None:
                delta_left = self._ticks_left - self._last_tick_left
                delta_right = self._ticks_right - self._last_tick_right

                actual_left_speed = delta_left / dt
                actual_right_speed = delta_right / dt

                error_left = self.target_ticks_per_sec - actual_left_speed
                error_right = self.target_ticks_per_sec - actual_right_speed

                control_left = self.left_pid.update(error_left, dt)
                control_right = self.right_pid.update(error_right, dt)

                # Saturate control signals
                control_left = max(min(control_left, 1.0), -1.0)
                control_right = max(min(control_right, 1.0), -1.0)

                # Send command to wheels
                self._publisher.publish(WheelsCmdStamped(vel_left=control_left, vel_right=control_right))

                rospy.loginfo(f"[PID] L_error: {error_left:.2f}, R_error: {error_right:.2f}")
                rospy.loginfo(f"[CMD] L_cmd: {control_left:.2f}, R_cmd: {control_right:.2f}")
                rospy.loginfo(f"[ENC] L_speed: {actual_left_speed:.2f}, R_speed: {actual_right_speed:.2f}")

                self._last_tick_left = self._ticks_left
                self._last_tick_right = self._ticks_right
            else:
                rospy.loginfo("[PID] Waiting for encoder data...")

            rate.sleep()

if __name__ == '__main__':
    node = EncoderPIDNode(node_name='encoder_pid_node')
    node.run()


   ''' 