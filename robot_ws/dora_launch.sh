#!/bin/bash

# Exit immediately if any command exits with a non-zero status
set -e

# Change permissions for serial devices before proceeding
echo "Setting permissions for /dev/ttyS0 and /dev/ttyUSB0..."
sudo chmod 666 /dev/ttyS0
sudo chmod 666 /dev/ttyUSB0

# Launch ROS2 nodes in the background with proper handling
echo "Launching SLAM Toolbox..."
ros2 launch slam_toolbox online_async_launch.py &
slam_pid=$!

echo "Launching Encoder Odometry..."
ros2 launch encoder_odometry robot_launch.py &
encoder_pid=$!

echo "Launching RPLIDAR..."
ros2 launch rplidar_ros rplidar_a2m8_launch.py &
rplidar_pid=$!

# Wait a short time to ensure all nodes are up before running wheel controller
sleep 5

echo "Running Wheel Controller..."
ros2 run wheel_controller wheel_controller &
wheel_controller_pid=$!

# Function to handle cleanup when script is interrupted or exits
cleanup() {
    echo "Stopping all processes..."
    kill $slam_pid $encoder_pid $rplidar_pid $wheel_controller_pid
    wait $slam_pid $encoder_pid $rplidar_pid $wheel_controller_pid 2>/dev/null
    echo "All processes stopped."
}

# Trap script termination (e.g., Ctrl+C) to call cleanup
trap cleanup EXIT

# Wait for all background processes to complete
wait
