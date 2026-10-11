import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Namespace argument
    robot_namespace = LaunchConfiguration('namespace', default='JeffCreate3')
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='JeffCreate3',
        description='Top-level robot namespace'
    )

    # 2. Config path
    config = os.path.join(
        get_package_share_directory('nuc_robot_bringup'),
        'config',
        'realsense_config.yaml'
    )

    # 3. RealSense Driver Node
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        namespace=robot_namespace,
        parameters=[config],
        remappings=[
            ('/tf', ['/', robot_namespace, '/tf']),
            ('/tf_static', ['/', robot_namespace, '/tf_static']),
        ],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        namespace_arg,
        realsense_node
    ])