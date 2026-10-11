import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare top-level namespace argument
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='JeffCreate3',
        description='Top-level robot namespace'
    )
    robot_namespace = LaunchConfiguration('namespace')

    # Path to Xacro file
    xacro_file = PathJoinSubstitution([
        FindPackageShare('jeff_create3_robot_description'),
        'urdf',
        'jeff_create3.urdf.xacro'
    ])

    # Evaluate Xacro to XML string with runtime namespace mapping
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' namespace:=', robot_namespace]),
        value_type=str
    )

    # Centralized Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_namespace,
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }],
        remappings=[
            ('/tf', ['/', robot_namespace, '/tf']),
            ('/tf_static', ['/', robot_namespace, '/tf_static']),
            ('joint_states', ['/', robot_namespace, '/joint_states'])
        ],
        output='screen'
    )

    return LaunchDescription([
        namespace_arg,
        robot_state_publisher_node
    ])