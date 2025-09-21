# honest_launcher.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    honest_pub = Node(
        package='honest_package',
        executable='honest_publisher_node',
        name='honest_publisher',
        output='screen',
    )

    honest_sub = Node(
        package='honest_subscriber_pkg',
        executable='honest_subscriber_node',
        name='honest_subscriber',
        output='screen',
    )

    return LaunchDescription([honest_pub, honest_sub])

