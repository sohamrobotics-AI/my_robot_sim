#!/usr/bin/env python3
"""Sim-only glue between Gazebo and the rest of the stack.

1. Gazebo publishes zero covariances for wheel odometry and IMU; robot_localization needs
   non-zero values, so they are filled in here.
2. Gazebo stamps sensor data with its own internal frame names (e.g.
   'my_robot/base_footprint/camera') because fixed-joint links get merged. Those frames are not
   in the TF tree, so the correct URDF frame names are re-stamped here.

  /wheel/odom_raw -> /wheel/odom          /imu/data_raw -> /imu/data
  /camera/image_raw       -> /camera/image
  /camera/depth_image_raw -> /camera/depth_image
  /camera/camera_info_raw -> /camera/camera_info
"""
from functools import partial

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, Imu

OPTICAL_FRAME = 'camera_optical_link'
IMU_FRAME = 'imu_link'


def diag(v, n):
    c = [0.0] * (n * n)
    for i, x in enumerate(v):
        c[i * n + i] = x
    return c


class SimGlue(Node):
    def __init__(self):
        super().__init__('sim_covariance_fix')
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odom', 20)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 50)
        self.create_subscription(Odometry, '/wheel/odom_raw', self.on_odom, 20)
        self.create_subscription(Imu, '/imu/data_raw', self.on_imu, 50)

        for topic, msg_type in (('/camera/image', Image),
                                ('/camera/depth_image', Image),
                                ('/camera/camera_info', CameraInfo)):
            pub = self.create_publisher(msg_type, topic, 5)
            self.create_subscription(msg_type, topic + '_raw',
                                     partial(self.relay_camera, pub), 5)

    def relay_camera(self, pub, msg):
        msg.header.frame_id = OPTICAL_FRAME
        pub.publish(msg)

    def on_odom(self, m):
        m.pose.covariance = diag([1e-3, 1e-3, 1e6, 1e6, 1e6, 1e-2], 6)
        m.twist.covariance = diag([1e-3, 1e-3, 1e6, 1e6, 1e6, 1e-2], 6)
        self.odom_pub.publish(m)

    def on_imu(self, m):
        m.header.frame_id = IMU_FRAME
        m.orientation_covariance = diag([1e-3, 1e-3, 1e-3], 3)
        m.angular_velocity_covariance = diag([4e-6, 4e-6, 4e-6], 3)
        m.linear_acceleration_covariance = diag([4e-4, 4e-4, 4e-4], 3)
        self.imu_pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(SimGlue())


if __name__ == '__main__':
    main()
