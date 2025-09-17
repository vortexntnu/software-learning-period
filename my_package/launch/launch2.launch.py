from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='new_node',
            name='new_node',
            output='screen',
            parameters=[{'param_name': 'param_value'}],
            remappings=[('/input_topic', '/output_topic')]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('simple_publisher'), 'launch', 'simple_publisher.launch.py'
                )  
            )
        )
    ])