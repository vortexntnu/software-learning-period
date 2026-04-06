import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('object_detection_example'),
        'config',
        'object_detection_example.yaml'
    )

    return LaunchDescription([
        Node(
            package='object_detection_example',
            executable='object_detection_example_node',
            name='object_detection_example_node',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='object_detection_example',
            executable='image_republisher_node',
            name='image_republisher_node',
            parameters=[config],
            output='screen',
        )
    ])
