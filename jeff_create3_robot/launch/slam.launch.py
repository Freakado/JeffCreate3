import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.descriptions import ParameterFile
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg_dir = get_package_share_directory('jeff_create3_robot')
    default_params_file = os.path.join(pkg_dir, 'config', 'slam_params.yaml')

    # Arguments
    namespace_arg = DeclareLaunchArgument('namespace', default_value='JeffCreate3')
    autostart_arg = DeclareLaunchArgument('autostart', default_value='true')
    use_lifecycle_manager_arg = DeclareLaunchArgument('use_lifecycle_manager', default_value='false')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    params_file_arg = DeclareLaunchArgument('params_file', default_value=default_params_file)

    namespace = LaunchConfiguration('namespace')
    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration('use_lifecycle_manager')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    params_file_w_subst = ParameterFile(params_file, allow_substs=True)

    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            params_file_w_subst,
            {
                'use_lifecycle_manager': use_lifecycle_manager,
                'use_sim_time': use_sim_time,
            },
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/scan', 'scan'),
            ('/map', 'map'),
            ('/map_metadata', 'map_metadata'),
        ],
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
        ),
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[LifecycleLaunch] Slamtoolbox node is activating.'),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
        ),
    )

    return LaunchDescription([
        namespace_arg,
        autostart_arg,
        use_lifecycle_manager_arg,
        use_sim_time_arg,
        params_file_arg,
        slam_node,
        configure_event,
        activate_event,
    ])