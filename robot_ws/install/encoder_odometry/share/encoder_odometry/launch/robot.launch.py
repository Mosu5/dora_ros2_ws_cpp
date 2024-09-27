from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Odometry node for holonomic robot
        Node(
            package='encoder_odometry',
            executable='encoder_to_odom',
            name='encoder_to_odom',
            output='screen',
            parameters=[
                {'port': '/dev/ttyUSB0'},   # Serial port for Arduino
                {'baudrate': 115200},       # Baudrate for the serial communication
                {'wheel_radius': 0.05},     # Wheel radius in meters
                {'wheel_base_length': 0.153}, # Wheel base length in meters
                {'wheel_base_width': 0.362},  # Wheel base width in meters
            ]
        ),
        
        # Static transform publisher for LIDAR
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.06', '0', '0', '0', 'base_link', 'laser_frame'],
            output='screen',
        ),

        # You can add other nodes, like the LIDAR driver or SLAM node, here.
        # For example, you might include a SLAM algorithm like Gmapping or Cartographer:
        # Node(
        #     package='slam_toolbox',
        #     executable='async_slam_toolbox_node',
        #     name='slam_toolbox',
        #     output='screen',
        #     parameters=[{'use_sim_time': False}],
        # ),
    ])

