from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="my_package",
            executable="my_publisher_node",
            name="my_publisher",
            output="screen"
        ),
        Node(
            package="my_package2",
            executable="my_subscriber_node",
            name="my_subscriber",
            output="screen"
        )
    ])