import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():

    robot_namespace = 'JeffCreate3'

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value=robot_namespace,
        description='Top-level robot namespace'
    )

    # Robot State Publisher & Unified Kinematics ---
    description_package_dir = get_package_share_directory('jeff_create3_robot_description')
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_package_dir, 'launch', 'description.launch.py')
        ),
        launch_arguments={'namespace': robot_namespace}.items()
    )

    # Unity ROS TCP Endpoint ---
    tcp_endpoint_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nuc_robot_bringup'),
                'launch',
                'tcp_endpoint.launch.py'
            )
        ),
        launch_arguments={
            'namespace': robot_namespace,
            'tcp_ip': '0.0.0.0',
            'tcp_port': '10000',
            'node_name': 'unity_tcp_endpoint_control'
        }.items()
    )

    # Unity ROS TCP Endpoint ---
    tcp_endpoint_video_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nuc_robot_bringup'),
                'launch',
                'tcp_endpoint.launch.py'
            )
        ),
        launch_arguments={
            'namespace': robot_namespace,
            'tcp_ip': '0.0.0.0',
            'tcp_port': '10001',
            'node_name': 'unity_tcp_endpoint_video'
        }.items()
    )

    # RealSense Camera Stream ---
    realsense_package_dir = get_package_share_directory('nuc_robot_bringup')
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_package_dir, 'launch', 'realsense.launch.py')
        ),
        launch_arguments={'namespace': robot_namespace}.items()
    )

    # RPLiDAR C1 Driver ---
    rplidar_launch_path = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch',
        'rplidar_c1_launch.py'
    )
    rplidar_group = GroupAction(
        actions=[
            PushRosNamespace(robot_namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rplidar_launch_path),
                launch_arguments={
                    'serial_port': '/dev/ttyUSB0',
                    'frame_id': 'laser'
                }.items()
            )
        ]
    )

    # NUC Health Monitoring ---
    nuc_health_node = Node(
        package='nuc_robot_bringup',
        executable='cpu_tempnode',
        name='nuc_health_node',
        namespace=robot_namespace,
        remappings=[
            ('cpu_temperature', 'nuc_cpu_temperature'),
            ('/cpu_temperature', f'/{robot_namespace}/nuc_cpu_temperature'),
        ],
        output='screen'
    )

    robot_manager_node = Node(
        package='nuc_robot_bringup',
        executable='robot_manager_node',
        name='robot_manager_node',
        namespace=robot_namespace,
        output='screen'
    )

    # ROS 2 Domain Bridge (Domain 0 -> Domain 10) ---
    pacakge_dir = get_package_share_directory('nuc_robot_bringup')
    wifibridge_config_path = os.path.join(pacakge_dir, 'config', 'dds_eth_wifi_bridge.yaml')
    wifi_bridge_node = Node(
        package='domain_bridge',
        executable='domain_bridge',
        name='domain_wifi_bridge',
        namespace=robot_namespace,
        arguments=[wifibridge_config_path],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    usbbridge_config_path = os.path.join(pacakge_dir, 'config', 'dds_eth_usb_bridge.yaml')
    usb_bridge_node = Node(
        package='domain_bridge',
        executable='domain_bridge',
        name='domain_usb_bridge',
        namespace=robot_namespace,
        arguments=[usbbridge_config_path],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    puzzle_server = Node(
        package='jeff_create3_robot',
        executable='puzzle_server',
        name='puzzle_server_node',
        namespace=robot_namespace,
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        namespace_arg,
        description_launch,
        realsense_launch,
        rplidar_group,
        nuc_health_node,
        robot_manager_node,
        wifi_bridge_node,
        usb_bridge_node,
        tcp_endpoint_control_launch,
        tcp_endpoint_video_launch
    ])