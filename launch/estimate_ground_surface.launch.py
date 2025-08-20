#estimate_ground_surface.launch.py
import os
from ament_index_python.packages import get_package_share_directory # type: ignore
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument # type: ignore
from launch.substitutions import LaunchConfiguration, TextSubstitution # type: ignore
from launch_ros.actions import Node # type: ignore
def generate_launch_description():
    config_file_path = os.path.join(get_package_share_directory('estimate_ground_surface'), 'config', 'params.yaml')
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file_path,
        description='Path to the configuration file'
    )
    config_file = LaunchConfiguration('config_file')
    edge_node = Node(
        package='estimate_ground_surface',
        executable='estimate_ground_surface',
        parameters=[config_file],
        output='screen',
        name='estimate_ground_surface'
    )
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_livox',
        arguments=['1.0', '0', '-1.0', '0', '0', '0', 'base_link', 'livox_frame']
    )
    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(config_file_arg)
    ld.add_action(static_tf_node)
    ld.add_action(edge_node)
    return ld