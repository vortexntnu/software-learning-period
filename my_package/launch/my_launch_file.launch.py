# my_package/launch/my_launch_file.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_publisher_node',
            name='publisher',
            output='screen'
        ),
        Node(
            package='my_subscriber',
            executable='my_subscriber_node',
            name='subscriber',
            output='screen'
        ),
    ])
