#!/bin/bash
sudo chmod 666 /dev/ttyS0 &
sudo chmod 666 /dev/ttyUSB0 &
ros2 launch slam_toolbox online_async_launch.py & 
ros2 launch encoder_odometry robot_launch.py &
ros2 launch rplidar_ros rplidar_a2m8_launch.py &
ros2 run wheel_controller wheel_controller
 
