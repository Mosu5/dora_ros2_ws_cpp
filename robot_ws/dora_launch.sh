#!/bin/bash

# Exit immediately if any command exits with a non-zero status
set -e

# Change permissions for serial devices before proceeding
echo "Setting permissions for /dev/ttyS0 and /dev/ttyUSB0..."
sudo chmod 666 /dev/ttyS0
sudo chmod 666 /dev/ttyUSB0

echo "Running Wheel Controller..."
ros2 run wheel_controller wheel_controller &
wheel_controller_pid=$!


# Wait a short time to ensure all nodes are up before running wheel controller
sleep 10

echo "Launching RPLIDAR..."
ros2 launch rplidar_ros rplidar_a2m8_launch.py &
rplidar_pid=$!


# Function to handle cleanup when script is interrupted or exits
cleanup() {
    echo "Stopping all processes..."
    kill $encoder_pid $rplidar_pid $wheel_controller_pid
    wait $encoder_pid $rplidar_pid $wheel_controller_pid 2>/dev/null
    echo "All processes stopped."
}

# Trap script termination (e.g., Ctrl+C) to call cleanup
trap cleanup EXIT

# Wait for all background processes to complete
wait
