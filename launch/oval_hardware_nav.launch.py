"""Start Nav2 for a mapped physical course with a guarded velocity output."""

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


class HardwareRewrittenYaml(RewrittenYaml):
    """Allow whole-sequence rewrites for Jazzy velocity vector parameters."""

    def convert(self, text_value):
        if text_value.startswith('['):
            return json.loads(text_value)
        return super().convert(text_value)


def start_navigation(context):
    map_path = Path(LaunchConfiguration('map').perform(context)).expanduser().resolve()
    if not map_path.is_file():
        raise ValueError(f'Physical course map YAML does not exist: {map_path}')

    odom_topic = LaunchConfiguration('odom_topic').perform(context)
    if not odom_topic.startswith('/'):
        raise ValueError('odom_topic must be an absolute ROS topic name')

    profile = (Path(get_package_share_directory('ecd726_description')) /
               'oval_nav2' / 'oval_nav2_stable_042.yaml')
    params_file = HardwareRewrittenYaml(
        source_file=str(profile),
        param_rewrites={
            'odom_topic': odom_topic,
            'amcl.ros__parameters.set_initial_pose': 'false',
            'amcl.ros__parameters.always_reset_initial_pose': 'true',
            'amcl.ros__parameters.scan_topic': '/scan',
            'controller_server.ros__parameters.FollowPath.vx_max': '0.15',
            'velocity_smoother.ros__parameters.max_velocity': '[0.15, 0.0, 1.25]',
            'velocity_smoother.ros__parameters.min_velocity': '[0.0, 0.0, -1.25]',
            'velocity_smoother.ros__parameters.velocity_timeout': '0.25',
            'local_costmap.local_costmap.ros__parameters.voxel_layer.scan.topic': '/scan',
            'global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.topic': '/scan',
            'collision_monitor.ros__parameters.cmd_vel_out_topic': '/oval/cmd_vel_checked',
            'collision_monitor.ros__parameters.source_timeout': '0.3',
            'collision_monitor.ros__parameters.scan.topic': '/scan',
        },
        convert_types=True,
    )

    bringup = Path(get_package_share_directory('nav2_bringup')) / 'launch/bringup_launch.py'
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup)),
        launch_arguments={
            'map': str(map_path),
            'params_file': params_file,
            'slam': 'False',
            'use_sim_time': 'False',
        }.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Occupancy-map YAML for the physical course'),
        DeclareLaunchArgument('odom_topic', default_value='/odom',
                              description='Calibrated physical odometry topic'),
        OpaqueFunction(function=start_navigation),
    ])
