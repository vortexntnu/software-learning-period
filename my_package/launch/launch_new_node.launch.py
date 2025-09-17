from launch import LaunchDescription
from launch_ros.actions import Node

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
        Node(
            package='talker',
            executable='talker_node',
            name='talker',
            output='screen'
        ),
        Node(
            package='listener',
            executable='listener_node',
            name='listener',
            output='screen'
        )   
    ])