from launch import LaunchDescription
from launch_ros.actions import Node

import math

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_camera',
            arguments=['--x', '0.7', '--y', '0', '--z', '-0.2',
                       '--yaw', str(math.radians(90)),
                       '--pitch', str(math.radians(0)),
                       '--roll', str(math.radians(90)),
                       '--frame-id', 'base_link', '--child-frame-id', 'camera'],
        ),

    ])
