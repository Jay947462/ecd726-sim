"""Start the simulated rover, online SLAM, and the FollowPath command chain."""

from pathlib import Path
from shlex import quote

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from nav2_common.launch import RewrittenYaml
import yaml


def start_learning(context):
    share = Path(get_package_share_directory('ecd726_description'))
    track = LaunchConfiguration('track').perform(context)
    world = share / 'world' / f'ecd726_{track}_track.sdf'
    clock = {'use_sim_time': True}
    config = share / 'config'
    with (config / 'rover_sim.yaml').open(encoding='utf-8') as stream:
        rear_axle_x = -yaml.safe_load(stream)['wheelbase_m'] / 2

    robot = share / 'src' / 'ecd726_robot_main.sdf.xacro'
    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[clock, {'robot_description': ParameterValue(
            Command(['xacro ', quote(str(robot))]), value_type=str)}],
    )
    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', 'my_world', '-topic', '/robot_description',
                   '-name', 'ecd726_bot', '-y', '2.0', '-z', '0.095'],
    )
    rear_axle_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='rear_axle_tf',
        arguments=['--x', str(rear_axle_x), '--frame-id', 'base_link',
                   '--child-frame-id', 'rear_axle'],
        parameters=[clock],
    )
    odometry = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        parameters=[str(config / 'ekf.yaml'), clock],
    )
    slam_launch = Path(get_package_share_directory('slam_toolbox')) / 'launch' / 'online_async_launch.py'
    mapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(slam_launch)),
        launch_arguments={
            'slam_params_file': str(config / 'track_learning_slam.yaml'),
            'use_sim_time': 'true',
        }.items(),
    )

    # Keep MPPI's short-horizon collision costs tied to the current scan.
    # The learned map separately validates the closed reference at handoff.
    profile = RewrittenYaml(
        source_file=str(share / 'oval_nav2' / 'oval_nav2_stable_042.yaml'),
        param_rewrites={'local_costmap.local_costmap.ros__parameters.static_layer.enabled': 'false'},
        convert_types=True,
    )
    nav_params = [ParameterFile(profile, allow_substs=True), clock]
    remappings = [('cmd_vel', 'cmd_vel_nav')]
    controller = Node(
        package='nav2_controller', executable='controller_server', name='controller_server',
        output='screen', parameters=nav_params, remappings=remappings,
    )
    smoother = Node(
        package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
        output='screen', parameters=nav_params, remappings=remappings,
    )
    monitor = Node(
        package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor',
        output='screen', parameters=[*nav_params, {'base_shift_correction': False}],
    )
    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[clock, {'autostart': True,
                            'node_names': ['controller_server', 'velocity_smoother',
                                           'collision_monitor']}],
    )
    return [
        ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', '--headless-rendering', str(world)],
                       output='screen'),
        ExecuteProcess(cmd=['gz', 'sim', '-g'], condition=IfCondition(LaunchConfiguration('gui')),
                       output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': str(config / 'bridge_config.yaml')}]),
        state_publisher,
        spawn,
        rear_axle_tf,
        odometry,
        mapper,
        controller,
        smoother,
        monitor,
        lifecycle,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('track', default_value='circle', choices=['circle', 'oval'],
                              description='Simulation course fixture'),
        DeclareLaunchArgument('gui', default_value='false', choices=['true', 'false'],
                              description='Open a Gazebo GUI'),
        OpaqueFunction(function=start_learning),
    ])
