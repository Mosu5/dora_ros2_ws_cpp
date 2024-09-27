from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mecanum_controller',
            executable='mecanum_controller',
            name='mecanum_controller',
            output='screen',
            parameters=[
                {'port': '/dev/ttyS0'},  # Serial port for communication
                {'baudrate': 9600},      # Baudrate for serial communication
                {'wheel_radius': 0.08},  # Wheel radius in meters
                {'wheel_base_length': 0.153},  # Wheelbase length in meters
                {'wheel_base_width': 0.362}  # Wheelbase width in meters
            ]
        )
    ])
