"""Launch both starter nodes together, for Task 8 integration testing.

    ros2 launch transit_starter transit_starter.launch.py

Run this alongside `ros2 launch transit_sim transit_sim.launch.py`, which
must already be running so there is a city to draw into.

Both nodes are given config/transit_params.yaml, the same way vortex-auv's
los_guidance.launch.py hands its node guidance_params.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Build the launch description."""
    params = os.path.join(
        get_package_share_directory('transit_starter'),
        'config',
        'transit_params.yaml',
    )

    return LaunchDescription(
        [
            Node(
                package='transit_starter',
                executable='vehicle_node',
                name='vehicle_node',
                parameters=[params],
                output='screen',
            ),
            Node(
                package='transit_starter',
                executable='signal_node',
                name='signal_node',
                parameters=[params],
                output='screen',
            ),
        ]
    )
