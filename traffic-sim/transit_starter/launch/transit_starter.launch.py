"""Launch both starter nodes together, for Task 8 integration testing.

    ros2 launch transit_starter transit_starter.launch.py

Run this alongside `ros2 launch transit_sim transit_sim.launch.py` (which
must already be running so there's a city to draw into).
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Build the launch description."""
    return LaunchDescription(
        [
            Node(
                package='transit_starter',
                executable='vehicle_node',
                name='vehicle_node',
                output='screen',
            ),
            Node(
                package='transit_starter',
                executable='signal_node',
                name='signal_node',
                output='screen',
            ),
        ]
    )
