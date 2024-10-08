from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('rplidar_ros'), '/launch/rplidar_a2m8_launch.py'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('dora_robot_control'), '/launch/dora_robot_control_launch.py'])
        )
    ])