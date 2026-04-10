from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_camera',
            arguments=[ '--x', '0.7', '--y', '0', '--z', '-0.2',
                        '--yaw', '1.5708', '--pitch', '0', '--roll', '1.5708',
                        '--frame-id', 'base_link', '--child-frame-id', 'camera'],
        ),

    ])
