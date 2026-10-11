import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_namespace = 'JeffCreate3'

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value=robot_namespace,
        description='Top-level robot namespace'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start Nav2 lifecycle nodes'
    )

    enable_exploration_arg = DeclareLaunchArgument(
        'enable_exploration',
        default_value='true',
        description='Launch autonomous frontier explorer and auto-start trigger'
    )

    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('jeff_create3_robot'),
            'config',
            'nav2_params.yaml'
        ]),
        description='Full path to Nav2 parameter file'
    )

    ekf_params_arg = DeclareLaunchArgument(
        'ekf_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('jeff_create3_robot'),
            'config',
            'ekf.yaml'
        ]),
        description='Full path to EKF parameter file'
    )

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    enable_exploration = LaunchConfiguration('enable_exploration')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    ekf_params_file = LaunchConfiguration('ekf_params_file')

    namespaced_stack = GroupAction(
        actions=[
            PushRosNamespace(namespace),

            SetRemap(src='/tf', dst='tf'),
            SetRemap(src='/tf_static', dst='tf_static'),

            # 1. EKF Filter Node
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
                remappings=[
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                ]
            ),

            # 2. SLAM Mapping Pipeline
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('jeff_create3_robot'),
                        'launch',
                        'slam.launch.py'
                    )
                ),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                }.items()
            ),

            # 3. Nav2 Navigation Stack
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('nav2_bringup'),
                        'launch',
                        'navigation_launch.py'
                    )
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_params_file,
                    'autostart': autostart,
                    'use_composition': 'False',
                    'use_collision_monitor': 'True',
                    'use_docking_server': 'True',
                    'use_route_server': 'True',
                }.items()
            ),

            # 4. Twist Stamper Bridge
            Node(
                package='jeff_create3_robot',
                executable='twist_stamper',
                name='twist_stamper',
                output='screen'
            ),

            # 5. Frontier Explorer Node (Conditional)
            Node(
                package='jeff_create3_robot',
                executable='explorer_node',
                name='explorer_node',
                output='screen',
                condition=IfCondition(enable_exploration),
                parameters=[{
                    'base_frame': 'base_link',
                    'map_frame': 'map',
                    'step_distance': 1.2,
                    'max_distance': 2.5,
                    'obstacle_clearance': 0.6,
                    'manual_resume_mode': 'auto',
                    'auto_resume_delay_sec': 4.0,
                }],
                remappings=[
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                ]
            ),

            # 6. Hazard to Costmap Bridge
            Node(
                package='jeff_create3_robot',
                executable='hazard_to_pointcloud',
                name='hazard_to_pointcloud',
                output='screen',
                remappings=[
                    ('/hazard_detection', 'hazard_detection'),
                    ('/hazard_points', 'hazard_points'),
                ]
            ),

            # 7. RealSense PointCloud Downsampler
            Node(
                package='jeff_create3_robot',
                executable='hazard_depth_filter',
                name='depth_downsampler_node',
                output='screen',
            ),
        ]
    )

    # Delayed trigger for frontier explorer (Conditional)
    explore_trigger = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once', '-w', '0',
                    '--qos-durability', 'transient_local',
                    '/JeffCreate3/start_explore',
                    'std_msgs/msg/String',
                    'data: "start"'
                ],
                output='screen',
                condition=IfCondition(enable_exploration)
            )
        ]
    )

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        autostart_arg,
        enable_exploration_arg,
        nav2_params_arg,
        ekf_params_arg,
        namespaced_stack,
        explore_trigger,
    ])