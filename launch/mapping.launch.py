"""RTAB-Map visual SLAM (RGB-D) on top of the EKF odometry.

Data flow (same as the report):
  rgbd_odometry (visual odometry, /vo/odom) --yaw only--> EKF --> odom->base_footprint TF
  rtabmap (uses EKF odom TF + RGB-D) --> /map and map->odom TF (loop-closure correction)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('my_robot_sim')
    rviz_cfg = os.path.join(pkg, 'rviz', 'mapping.rviz')
    db_path = os.path.expanduser('~/.ros/my_robot_rtabmap.db')

    localization = LaunchConfiguration('localization')
    use_rviz = LaunchConfiguration('rviz')

    # ---- Visual odometry (RGB-D) -------------------------------------------
    rgbd_odom = Node(
        package='rtabmap_odom', executable='rgbd_odometry', name='rgbd_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',      # message frame only; TF is published by the EKF
            'publish_tf': False,
            'approx_sync': True,
            'queue_size': 10,
            'wait_for_transform': 0.5,
            'Reg/Force3DoF': 'true',      # planar robot
            'Vis/MinInliers': '12',
        }],
        remappings=[('rgb/image', '/camera/image'),
                    ('depth/image', '/camera/depth_image'),
                    ('rgb/camera_info', '/camera/camera_info'),
                    ('odom', '/vo/odom')])

    # ---- RTAB-Map SLAM ------------------------------------------------------
    rtabmap_params = {
        'use_sim_time': True,
        'frame_id': 'base_footprint',
        'odom_frame_id': 'odom',          # use TF odom->base_footprint from the EKF
        'map_frame_id': 'map',
        'publish_tf': True,               # map->odom
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_odom_info': False,
        'approx_sync': True,
        'queue_size': 10,
        'wait_for_transform': 0.5,
        'database_path': db_path,

        'Reg/Force3DoF': 'true',
        'Optimizer/Slam2D': 'true',
        'Rtabmap/DetectionRate': '2',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.05',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/OptimizeMaxError': '3.0',
        'Kp/MaxFeatures': '500',
        'Vis/MinInliers': '15',
        'Mem/NotLinkedNodesKept': 'false',

        # 2D occupancy grid from depth
        'Grid/FromDepth': 'true',
        'Grid/CellSize': '0.05',
        'Grid/RangeMin': '0.3',
        'Grid/RangeMax': '5.0',
        'Grid/MaxGroundHeight': '0.05',
        'Grid/MaxObstacleHeight': '1.6',
        'Grid/NormalsSegmentation': 'false',
        'Grid/RayTracing': 'true',
    }
    rtabmap_remaps = [('rgb/image', '/camera/image'),
                      ('depth/image', '/camera/depth_image'),
                      ('rgb/camera_info', '/camera/camera_info')]

    rtabmap_mapping = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
        parameters=[rtabmap_params], remappings=rtabmap_remaps,
        arguments=['-d'],                              # -d : start a NEW database
        condition=UnlessCondition(localization))

    rtabmap_localization = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
        parameters=[dict(rtabmap_params, **{'Mem/IncrementalMemory': 'false',
                                            'Mem/InitWMWithAllNodes': 'true'})],
        remappings=rtabmap_remaps,                     # re-use the saved database
        condition=IfCondition(localization))

    rviz = Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg],
                parameters=[{'use_sim_time': True}], condition=IfCondition(use_rviz))

    return LaunchDescription([
        DeclareLaunchArgument('localization', default_value='false',
                              description='true = localize in the saved map, no new mapping'),
        DeclareLaunchArgument('rviz', default_value='true'),
        rgbd_odom, rtabmap_mapping, rtabmap_localization, rviz,
    ])

