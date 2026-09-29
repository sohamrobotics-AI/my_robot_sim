"""Gazebo Fortress + robot + ROS<->Gazebo bridge + sensor covariance fix + EKF."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('my_robot_sim')
    xacro_file = os.path.join(pkg, 'urdf', 'my_robot.urdf.xacro')
    world = os.path.join(pkg, 'worlds', 'retail_store.sdf')
    ekf_yaml = os.path.join(pkg, 'config', 'ekf.yaml')
    gz_launch = os.path.join(get_package_share_directory('ros_gz_sim'),
                             'launch', 'gz_sim.launch.py')

    stereo = LaunchConfiguration('stereo')
    headless = LaunchConfiguration('headless')

    # Lets Gazebo resolve model://retail_textures/...
    models = os.path.join(pkg, 'models')
    prev = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    set_res = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH',
                                     models + (':' + prev if prev else ''))

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' stereo:=', stereo]), value_type=str)

    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': '-r ' + world}.items(),
        condition=UnlessCondition(headless))

    gz_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': '-r -s --headless-rendering ' + world}.items(),
        condition=IfCondition(headless))

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description,
                            'use_sim_time': True}],
               output='screen')

    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-name', 'my_robot', '-topic', 'robot_description',
                            '-x', '-5.0', '-y', '0.0', '-z', '0.05', '-Y', '0.0'])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/wheel/odom_raw@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/imu/data_raw@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ],
        # Camera data arrives with Gazebo's internal frame names; sim_covariance_fix.py
        # re-stamps it and publishes the final /camera/* topics.
        remappings=[('/camera/image', '/camera/image_raw'),
                    ('/camera/depth_image', '/camera/depth_image_raw'),
                    ('/camera/camera_info', '/camera/camera_info_raw')])

    stereo_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        condition=IfCondition(stereo), parameters=[{'use_sim_time': True}],
        arguments=[
            '/camera/infra1/image_rect_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/infra1/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/camera/infra2/image_rect_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/infra2/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ])

    cov_fix = Node(package='my_robot_sim', executable='sim_covariance_fix.py',
                   output='screen', parameters=[{'use_sim_time': True}])

    ekf = Node(package='robot_localization', executable='ekf_node',
               name='ekf_filter_node', output='screen', parameters=[ekf_yaml])

    # Coloured 3D point cloud built from depth + RGB (frame: camera_optical_link)
    pointcloud = ComposableNodeContainer(
        name='pointcloud_container', namespace='', package='rclcpp_components',
        executable='component_container', output='screen',
        composable_node_descriptions=[ComposableNode(
            package='depth_image_proc', plugin='depth_image_proc::PointCloudXyzrgbNode',
            name='point_cloud_xyzrgb',
            parameters=[{'use_sim_time': True}],
            remappings=[('rgb/camera_info', '/camera/camera_info'),
                        ('rgb/image_rect_color', '/camera/image'),
                        ('depth_registered/image_rect', '/camera/depth_image'),
                        ('points', '/camera/points_colored')])],
        condition=IfCondition(LaunchConfiguration('pointcloud')))

    return LaunchDescription([
        DeclareLaunchArgument('stereo', default_value='false',
                              description='Add IR stereo pair (Isaac ROS VSLAM only)'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run Gazebo server only, no GUI'),
        DeclareLaunchArgument('pointcloud', default_value='true',
                              description='Publish /camera/points_colored'),
        set_res, gz_gui, gz_headless,
        rsp, spawn, bridge, stereo_bridge, cov_fix, ekf, pointcloud,
    ])
