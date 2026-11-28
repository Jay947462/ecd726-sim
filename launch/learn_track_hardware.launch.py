"""Start wall-clock SLAM and collision-checked navigation for a physical course."""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from rclpy.validate_full_topic_name import validate_full_topic_name


def start_learning(context):
    odom_topic = LaunchConfiguration('odom_topic').perform(context)
    validate_full_topic_name(odom_topic)
    minimum_radius = float(LaunchConfiguration('min_turn_radius').perform(context))
    if not math.isfinite(minimum_radius) or minimum_radius < 0.65:
        raise ValueError('min_turn_radius must be a measured radius of at least 0.65 m')
    share = Path(get_package_share_directory('ecd726_description'))
    slam_launch = Path(get_package_share_directory('slam_toolbox')) / 'launch' / 'online_async_launch.py'
    mapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(slam_launch)),
        launch_arguments={
            'slam_params_file': str(share / 'config' / 'track_learning_hardware_slam.yaml'),
            'use_sim_time': 'false',
        }.items(),
    )

    # Use current scan obstacles for control while the map is being built.
    profile = RewrittenYaml(
        source_file=str(share / 'oval_nav2' / 'oval_nav2_stable_042.yaml'),
        param_rewrites={'local_costmap.local_costmap.ros__parameters.static_layer.enabled': 'false'},
        convert_types=True,
    )
    clock = {'use_sim_time': False}
    nav_params = [ParameterFile(profile, allow_substs=True), clock]
    remappings = [('cmd_vel', 'cmd_vel_nav')]
    controller = Node(
        package='nav2_controller', executable='controller_server', name='controller_server',
        output='screen', remappings=remappings,
        parameters=[*nav_params, {
            'odom_topic': odom_topic,
            'FollowPath.vx_max': 0.15,
            'FollowPath.AckermannConstraints.min_turning_r': minimum_radius,
        }],
    )
    smoother = Node(
        package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
        output='screen', remappings=remappings,
        parameters=[*nav_params, {
            'odom_topic': odom_topic,
            'max_velocity': [0.15, 0.0, 1.25],
            'min_velocity': [0.0, 0.0, -1.25],
            'velocity_timeout': 0.25,
        }],
    )
    monitor = Node(
        package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor',
        output='screen',
        parameters=[*nav_params, {
            'cmd_vel_out_topic': '/oval/cmd_vel_checked',
            'source_timeout': 0.30,
            'scan.topic': '/scan',
            # Fresh upstream zero commands are needed for the stopped arming
            # and mapping/replay handoff checks during one supervised session.
            'stop_pub_timeout': 3600.0,
        }],
    )
    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[clock, {'autostart': True,
                            'node_names': ['controller_server', 'velocity_smoother',
                                           'collision_monitor']}],
    )
    return [mapper, controller, smoother, monitor, lifecycle]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('min_turn_radius',
                              description='Larger measured left/right turn radius in metres, at least 0.65'),
        DeclareLaunchArgument('odom_topic', default_value='/odom',
                              description='Calibrated physical odometry topic'),
        OpaqueFunction(function=start_learning),
    ])
