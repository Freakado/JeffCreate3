from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap


def generate_launch_description():
    robot_namespace = 'JeffCreate3'

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value=robot_namespace,
        description='Top-level robot namespace'
    )

    trial_schedule_arg = DeclareLaunchArgument(
        'trial',
        default_value='DEFAULT',
        description='Trial puzzle sequence pattern: DEFAULT, A, B, or C'
    )

    bypass_nav_arg = DeclareLaunchArgument(
        'bypass_nav',
        default_value='false',
        description='Bypass Nav2 driving for static bench testing'
    )

    standalone_test_mode_arg = DeclareLaunchArgument(
        'standalone_test_mode',
        default_value='false',
        description='Run puzzles without requiring exploration stack'
    )

    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='map',
        description='Frame to transform detected tags into (e.g. map, odom, or base_link)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    target_frame = LaunchConfiguration('target_frame')
    trial_schedule = LaunchConfiguration('trial')
    bypass_nav = LaunchConfiguration('bypass_nav')
    standalone_test_mode = LaunchConfiguration('standalone_test_mode')

    # Pre-emptively kill any lingering process holding port 8000
    free_port_process = ExecuteProcess(
        cmd=['bash', '-c', 'fuser -k 8000/tcp || true'],
        output='log'
    )

    namespaced_stack = GroupAction(
        actions=[
            PushRosNamespace(namespace),

            SetRemap(src='/tf', dst='tf'),
            SetRemap(src='/tf_static', dst='tf_static'),

            # 1. Artifact Inspector Node
            Node(
                package='jeff_create3_robot',
                executable='artifact_inspector_node',
                name='artifact_inspector_node',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'bypass_nav': bypass_nav,
                    'standalone_test_mode': standalone_test_mode,
                    'target_frame': target_frame,
                }],
            ),

            # 2. Puzzle Web Server Node
            Node(
                package='jeff_create3_robot',
                executable='puzzle_server_node',
                name='puzzle_server_node',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'standalone_test_mode': standalone_test_mode,
                    'trial_schedule': trial_schedule,
                }],
                respawn=False
            ),
        ]
    )

    return LaunchDescription([
        namespace_arg,
        trial_schedule_arg,
        bypass_nav_arg,
        standalone_test_mode_arg,
        target_frame_arg,
        use_sim_time_arg,
        free_port_process,
        namespaced_stack,
    ])