import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
import math

def generate_launch_description():

    urdf_file_name = 'person.urdf.xacro'
    urdf_path = os.path.join(
        get_package_share_directory('frames'),
        'description',
        urdf_file_name)

    doc = xacro.parse(open(urdf_path))
    xacro.process_doc(doc)
    robot_desc = doc.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}])

    # Transform from Base Link (NED) to Person (ENU)
    base_to_person_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_person_enu',
        arguments=['--x', '2.0', '--y', '0', '--z', '0',
                   '--yaw', str(math.pi/2), '--pitch', '0', '--roll', str(math.pi),
                   '--frame-id', 'base_link', '--child-frame-id', 'person_enu_link']
    )
  

    return LaunchDescription([
        robot_state_publisher_node,
        base_to_person_node,
    ])
