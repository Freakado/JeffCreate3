import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # ---------------------------------------------------------
    # Launch Arguments
    # ---------------------------------------------------------
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='JeffCreate3',
        description='Robot namespace for the TCP endpoint node and relative topics'
    )

    tcp_ip_arg = DeclareLaunchArgument(
        'tcp_ip',
        default_value='0.0.0.0',
        description='IP address for the TCP endpoint to bind to (0.0.0.0 binds to all interfaces including 192.168.1.100)'
    )

    tcp_port_arg = DeclareLaunchArgument(
        'tcp_port',
        default_value='10000',
        description='TCP port for Unity connection'
    )

    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='unity_endpoint',
        description='Node name for the ros_tcp_endpoint server'
    )

    # ---------------------------------------------------------
    # Launch Configurations
    # ---------------------------------------------------------
    namespace = LaunchConfiguration('namespace')
    tcp_ip = LaunchConfiguration('tcp_ip')
    tcp_port = LaunchConfiguration('tcp_port')
    node_name = LaunchConfiguration('node_name')

    # ---------------------------------------------------------
    # Endpoint Node Definition
    # ---------------------------------------------------------
    endpoint_group = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            Node(
                package='ros_tcp_endpoint',
                executable='default_server_endpoint',
                name=node_name,
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[
                    {'ROS_IP': tcp_ip},
                    {'ROS_TCP_PORT': tcp_port},
                ],
                emulate_tty=True,
            )
        ]
    )

    return LaunchDescription([
        namespace_arg,
        tcp_ip_arg,
        tcp_port_arg,
        node_name_arg,
        endpoint_group,
    ])