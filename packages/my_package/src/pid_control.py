#!/usr/bin/env python3

import cv2 as cv

import numpy as np

camera = cv.VideoCapture(0)

if not camera.isOpened():

    print("Error: Could not open camera.")

    exit()

class PIDController:

    def __init__(self, kp, ki, kd):

        self.kp = kp

        self.ki = ki

        self.kd = kd

        self.previous_error = 0

        self.integral = 0

    def reset(self):

        self.previous_error = 0

        self.integral = 0

    def compute(self, error, dt):

        self.integral += error * dt

        derivative = (error - self.previous_error) / dt if dt > 0 else 0


        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        self.previous_error = error

        return output


class LineFollowerRobot:

    def __init__(self, base_speed, pid_controller):

        self.base_speed = base_speed  # Speed when perfectly aligned

        self.pid = pid_controller

    def follow_line(self, line_error, dt):

        correction = self.pid.compute(line_error, dt)

        # Adjust wheel speeds

        left_speed = self.base_speed - correction

        right_speed = self.base_speed + correction

        # Optionally limit speeds (depends on your hardware)

        #left_speed = max(min(left_speed, 1.0), -1.0)

        #right_speed = max(min(right_speed, 1.0), -1.0)

        return left_speed, right_speed


# Example usage

if __name__ == "__main__":

    import time

    # PID Gains (You will need to tune these!)

    KP = 0.5  # Proportional gain

    KI = 0.05  # Integral gain

    KD = 0.1  # Derivative gain

    pid = PIDController(KP, KI, KD)

    robot = LineFollowerRobot(base_speed=0.5, pid_controller=pid)

    previous_time = time.time()

    while True:

        # --- SENSOR READING ---

        # Suppose we get a line position error here (-1 to 1)

        # Negative means line is to the left, positive to the right

        line_error = get_line_error(camera)  # <- you must implement this based on your sensors

        current_time = time.time()

        dt = current_time - previous_time

        previous_time = current_time

        # --- PID CONTROL ---

        left_speed, right_speed = robot.follow_line(line_error, dt)

        # --- MOTOR COMMAND ---

        set_wheel_speeds(left_speed, right_speed)  # <- you must implement this too

        time.sleep(0.01)  # Small delay to prevent 100% CPU usage


# Placeholder functions

def get_line_error(video):

    ret,frame= video.read()

    if not ret:

        print("Error: Could not read frame from camera.")

        return 0

    imgray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    im2, contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    return 0


def set_wheel_speeds(left, right):

    # Replace with your motor control code

    print(f"Left wheel speed: {left:.2f}, Right wheel speed: {right:.2f}")
 
