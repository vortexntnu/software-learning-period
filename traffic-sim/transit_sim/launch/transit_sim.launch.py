"""Launch the city: the sim, the map frame, and the Foxglove bridge.

    ros2 launch transit_sim transit_sim.launch.py

Then point Foxglove at ws://localhost:8765. Set `bridge:=false` if you would
rather view the same markers in RViz2.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Build the launch description."""
    bridge = LaunchConfiguration('bridge')
    port = LaunchConfiguration('port')
    publish_rate = LaunchConfiguration('publish_rate')

    bridge_launch = PathJoinSubstitution(
        [
            FindPackageShare('foxglove_bridge'),
            'launch',
            'foxglove_bridge_launch.xml',
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'bridge',
                default_value='true',
                description='Start foxglove_bridge. Set false to use RViz2 instead.',
            ),
            DeclareLaunchArgument(
                'port',
                default_value='8765',
                description='Port the Foxglove bridge listens on.',
            ),
            DeclareLaunchArgument(
                'publish_rate',
                default_value='20.0',
                description='How often the sim redraws, in Hz.',
            ),
            Node(
                package='transit_sim',
                executable='transit_sim',
                name='transit_sim',
                output='screen',
                parameters=[{'publish_rate': publish_rate}],
            ),
            # Foxglove's 3D panel needs a transform tree before it will render
            # markers, so publish an identity transform for the map frame.
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_frame',
                output='log',
                arguments=[
                    '--x',
                    '0',
                    '--y',
                    '0',
                    '--z',
                    '0',
                    '--roll',
                    '0',
                    '--pitch',
                    '0',
                    '--yaw',
                    '0',
                    '--frame-id',
                    'map',
                    '--child-frame-id',
                    'city',
                ],
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(bridge_launch),
                condition=IfCondition(bridge),
                launch_arguments={'port': port}.items(),
            ),
        ]
    )
