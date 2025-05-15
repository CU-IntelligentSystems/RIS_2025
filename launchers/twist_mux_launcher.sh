#!/bin/bash

source /environment.sh

# Initialize Duckietown launch process
dt-launchfile-init

# Get vehicle name from environment
VEHICLE_NAME=${VEHICLE_NAME:-"default_bot"}

# Launch nodes in background and save their PIDs
rosrun my_package camera_reader_node.py &
PID_CAMERA=$!

rosrun my_package stop_sequence.py &
PID_SUBSCRIBER=$!

rosrun my_package pid_node.py &
PID_PUBLISHER=$!

rosrun my_package twist_mux_selector.py &
PID_SELECTOR=$!

# Define cleanup function to run on Ctrl+C
cleanup() {
    echo "[Shutdown] Stopping motors and resetting LEDs..."

    # Publish zero velocity (non-blocking)
    rostopic pub /$VEHICLE_NAME/wheels_driver_node/wheels_cmd duckietown_msgs/WheelsCmdStamped "header:
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
vel_left: 0.0
vel_right: 0.0" --once &

    # Publish LED reset (non-blocking)
    rostopic pub /$VEHICLE_NAME/led_emitter_node/led_pattern duckietown_msgs/LEDPattern "rgb_vals:
- {r: 1.0, g: 1.0, b: 1.0, a: 1.0}
- {r: 1.0, g: 1.0, b: 1.0, a: 1.0}
- {r: 1.0, g: 1.0, b: 1.0, a: 1.0}
- {r: 1.0, g: 1.0, b: 1.0, a: 1.0}
- {r: 1.0, g: 1.0, b: 1.0, a: 1.0}
frequency: 0.0
frequency_mask: [0, 0, 0, 0, 0]" --once &

    echo "[Shutdown] Terminating nodes..."
    kill $PID_CAMERA $PID_SUBSCRIBER $PID_PUBLISHER $PID_SELECTOR 2>/dev/null
    wait

    echo "[Shutdown] Cleanup complete."
}

# Catch Ctrl+C and call cleanup
trap cleanup SIGINT

# Wait for all background nodes to finish
wait
