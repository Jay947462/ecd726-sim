from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# this file starts an rtabmap node for mapping, for the simulation

def generate_launch_description():
    params = {
        'use_sim_time': True,
        
        # reference existing tf and create new tf that RVIZ will display
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',        # take odometry from TF
        # 'map_frame_id': 'map',
        # 'publish_tf': True,             # publish map -> odom
        
        # what sensors to use
        'subscribe_scan': True,
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_rgbd': False,
        
        # RTAB-Map's own parameters are strings:
        'Reg/Strategy': '1',            # ICP (lidar scan matching)
        'Reg/Force3DoF': 'true',        # 2D: x, y, yaw only
        # 'RGBD/NeighborLinkRefining': 'true',
        # 'RGBD/ProximityBySpace': 'true',
        # 'RGBD/LinearUpdate': '0.05',
        # 'RGBD/AngularUpdate': '0.05',
        'Grid/Sensor': '0',             # build the grid from the laser scan
        # 'Icp/VoxelSize': '0.05',
        # 'Icp/MaxCorrespondenceDistance': '0.1',
        # 'Icp/PointToPlane': 'false',
        # 'Optimizer/GravitySigma': '0',
    }
    
    remappings = [('scan', '/scan')]

    return LaunchDescription([
        Node(package='rtabmap_slam', executable='rtabmap', output='screen',
             parameters=[params], remappings=remappings,
             arguments=['-d'],  # this deletes the old saved map
             condition=UnlessCondition(localization)),
    ])