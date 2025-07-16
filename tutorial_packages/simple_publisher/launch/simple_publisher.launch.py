from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
   
    # Get the path to the config file
    config_path = os.path.join(
        get_package_share_directory('simple_publisher'),
        'config',
        'simple_publisher_config.yaml'
    )

    # Declare a launch argument that the launch file can accept from the user
    # when launching the file with my_arg:=my_value
    node_name_arg = DeclareLaunchArgument(
        'simple_publisher_name',
        default_value='simple_publisher',
        description='Name of the simple_publisher node'
    )

    # LaunchConfiguration allows us to use the value of the argument
    # in the Node below
    simple_publisher_name = LaunchConfiguration('simple_publisher_name')

    return LaunchDescription([
        
        # package: Name of the package the node is located in
        # executable: The name of the compiled executable as defined in the CMakeLists.txt
        # name: Name of the node once it's launched
        # output: "screen" means that the node will print to console(terminal)
        node_name_arg, # The launch argument has to be returned
        # The launch argument has to be returned above where the launchconfiguration is used
        Node(
            package='talker',
            executable='talker_node',
            name='talker',
            output='screen'
        ),
        Node(
            package='simple_publisher',
            executable='simple_publisher',
            name=simple_publisher_name,
            parameters=[config_path],
            output='screen'
        )
    ])
