"""Launch the standalone Theta* planner node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('theta_star_planner_node')
    default_params = os.path.join(pkg_share, 'config', 'theta_star_planner_node.yaml')

    params_file = LaunchConfiguration('params_file')
    action = LaunchConfiguration('action')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to the parameter file for theta_star_planner_node',
        ),
        DeclareLaunchArgument(
            'action',
            default_value='/theta_star/compute_path_to_pose',
            description='ComputePathToPose action name to serve. smac_planner_node is '
                        'launched on /hybrid/compute_path_to_pose; pass that here to use '
                        'Theta* as a drop-in replacement for it.',
        ),
        Node(
            package='theta_star_planner_node',
            executable='theta_star_planner_node',
            name='theta_star_planner_node',
            output='screen',
            parameters=[params_file],
            remappings=[
                ('compute_path_to_pose', action),
            ]
        ),
    ])
