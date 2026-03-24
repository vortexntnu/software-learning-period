from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
   
    # Get the path to the config file
    config_path = os.path.join(
        get_package_share_directory('my_package'),
        'config',
        'my_package.yaml'
    )

    # Declare a launch argument that the launch file can accept from the user
    # when launching the file with my_arg:=my_value
    node_name_arg = DeclareLaunchArgument(
        'publisher_name',
        default_value='my_publisher_node',
        description='Name of the publisher node'
    )

    # LaunchConfiguration allows us to use the value of the argument
    # in the Node below
    publisher_name = LaunchConfiguration('publisher_name')

    return LaunchDescription([
        
        # package: Name of the package the node is located in
        # executable: The name of the compiled executable as defined in the CMakeLists.txt
        # name: Name of the node once it's launched
        # output: "screen" means that the node will print to console(terminal)
        node_name_arg, # The launch argument has to be returned
        # The launch argument has to be returned above where the launchconfiguration is used
        Node(
            package='my_package',
            executable='my_publisher',
            name='publisher_name',
            output='screen'
        ),
    ])