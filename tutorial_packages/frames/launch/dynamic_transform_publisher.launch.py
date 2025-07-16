from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    frames_launch_dir = os.path.join(
        get_package_share_directory('frames'),
        'launch'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(frames_launch_dir, 'static_frames.launch.py')
            )
        ),

        Node(
            package='frames',
            executable='dynamic_transform_publisher_node',
            name='dynamic_transform_publisher'
        )
    ])