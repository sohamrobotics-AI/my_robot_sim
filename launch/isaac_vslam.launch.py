"""OPTIONAL: Isaac ROS Visual SLAM (cuVSLAM) as the visual-odometry source instead of rgbd_odometry.

Needs an NVIDIA GPU + the Isaac ROS dev container (ROS 2 Humble). Run Gazebo with stereo:=true.
Output is remapped to /vo/odom, so ekf.yaml (yaw-only fusion) works unchanged.
NOTE: parameter names follow the Isaac ROS Visual SLAM docs; they change between releases,
so check them against the version you install.
"""
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    vslam = ComposableNode(
        name='visual_slam_node', package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'use_sim_time': True,
            'enable_image_denoising': False,
            'rectified_images': True,
            'enable_imu_fusion': True,
            'gyro_noise_density': 0.002,
            'gyro_random_walk': 0.0002,
            'accel_noise_density': 0.02,
            'accel_random_walk': 0.002,
            'calibration_frequency': 100.0,
            'base_frame': 'base_footprint',
            'imu_frame': 'imu_link',
            'camera_optical_frames': ['camera_infra1_optical_link',
                                      'camera_infra2_optical_link'],
            'publish_odom_to_base_tf': False,     # the EKF owns odom->base_footprint
            'publish_map_to_odom_tf': False,      # rtabmap owns map->odom
            'enable_slam_visualization': True,
        }],
        remappings=[
            ('visual_slam/image_0', '/camera/infra1/image_rect_raw'),
            ('visual_slam/camera_info_0', '/camera/infra1/camera_info'),
            ('visual_slam/image_1', '/camera/infra2/image_rect_raw'),
            ('visual_slam/camera_info_1', '/camera/infra2/camera_info'),
            ('visual_slam/imu', '/imu/data'),
            ('visual_slam/tracking/odometry', '/vo/odom'),
        ])

    container = ComposableNodeContainer(
        name='visual_slam_container', namespace='', package='rclcpp_components',
        executable='component_container_mt', composable_node_descriptions=[vslam],
        output='screen')
    return LaunchDescription([container])
