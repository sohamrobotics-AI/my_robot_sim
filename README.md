# visual_slam_diff_drive_robot

ROS 2 **Jazzy** + Gazebo **Harmonic** simulation of a differential-drive robot with an RGB-D
camera and IMU. Wheel encoders and the IMU are fused with visual odometry in an EKF
(`robot_localization`); visual odometry contributes **yaw only**, so featureless walls cannot
corrupt the position estimate. RTAB-Map builds a 2D occupancy map of the building and a 3D
point cloud, all in simulation.

## Features

- Robot description as xacro (base, two drive wheels, caster, sensor mast, RGB-D camera, IMU)
- Retail-store Gazebo world with textured shelves and one deliberately plain white wall
- ROS <-> Gazebo bridge, sensor covariance and frame fix-ups for simulation
- EKF sensor fusion: wheel velocity + IMU yaw rate + visual-odometry yaw
- RTAB-Map RGB-D visual SLAM: 2D map, 3D cloud map, loop closure, localization mode
- Two RViz windows: the 2D building map, and the 3D point clouds

## Requirements

Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic.

```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-xacro \
  ros-jazzy-robot-localization ros-jazzy-rtabmap-ros ros-jazzy-depth-image-proc \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-nav2-map-server ros-jazzy-tf2-tools
```

## Build

The repository folder can have any name; the ROS package inside is called `my_robot_sim`.

```bash
mkdir -p ~/ws/src && cd ~/ws/src/my_robot_sim
git clone https://github.com/<sohamrobotics-AI>/visual_slam_diff_drive_robot.git
cd ~/ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run

Terminal 1: simulation (Gazebo, robot, bridge, EKF, point cloud)

```bash
ros2 launch my_robot_sim sim.launch.py
```

Terminal 2: visual SLAM and the two RViz windows

```bash
ros2 launch my_robot_sim mapping.launch.py
```

Terminal 3: drive the robot

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.5
```

Drive slowly and do a full loop around the shelf block so RTAB-Map can close the loop.

## Save the map

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/retail_store
```

This writes `retail_store.yaml` and `retail_store.pgm` for Nav2. RTAB-Map also stores its
database at `~/.ros/my_robot_rtabmap.db`.

## Localize in a saved map

```bash
ros2 launch my_robot_sim sim.launch.py
ros2 launch my_robot_sim mapping.launch.py localization:=true
```

## Launch options

| Launch file | Argument | Default | Effect |
|---|---|---|---|
| `sim.launch.py` | `headless` | `false` | Gazebo server only, no GUI |
| `sim.launch.py` | `pointcloud` | `true` | Publish `/camera/points_colored` |
| `sim.launch.py` | `stereo` | `false` | Add an IR stereo pair (Isaac ROS VSLAM only) |
| `mapping.launch.py` | `localization` | `false` | Localize in the saved database instead of mapping |
| `mapping.launch.py` | `rviz` | `true` | Open the two RViz windows |

## Architecture

```
Gazebo (RGB-D camera, IMU, diff drive)
  -> ros_gz_bridge -> sim_covariance_fix.py (frame names + covariances)
      wheel odom + IMU  ------------------------------\
      rgbd_odometry (/vo/odom, yaw only, differential) -> EKF -> odom -> base_footprint
      RTAB-Map (RGB-D + EKF odom) -> /map, /cloud_map, map -> odom
```

TF tree: `map -> odom -> base_footprint -> base_link -> ... -> camera_optical_link`.

## Repository layout

```
urdf/      xacro robot description (+ Gazebo plugins and sensors)
worlds/    retail_store.sdf
models/    texture used by the world
config/    ekf.yaml
launch/    sim.launch.py, mapping.launch.py, isaac_vslam.launch.py
rviz/      mapping_map.rviz (2D map), mapping_3d_cloud.rviz (3D point cloud)
scripts/   sim_covariance_fix.py
```

## Optional: Isaac ROS Visual SLAM

`launch/isaac_vslam.launch.py` swaps RTAB-Map's visual odometry for Isaac ROS Visual SLAM.
It needs an NVIDIA GPU and the Isaac ROS environment, and Gazebo started with `stereo:=true`.
It publishes to the same `/vo/odom` topic, so the EKF config is unchanged. Untested.

## Troubleshooting

- **Black or empty camera images:** Gazebo rendering needs OpenGL. In a VM try
  `LIBGL_ALWAYS_SOFTWARE=1`, or lower the camera update rate in the xacro.
- **RViz shows "No transform from [map]":** RTAB-Map has not published `map -> odom` yet.
  Drive a few metres.
- **Visual odometry reports "lost":** drive slower and check `/camera/image`.
- **Check frames:** `ros2 topic echo /camera/image --field header.frame_id --once` should print
  `camera_optical_link`.

## Next steps

- Nav2 (planner, controller, behaviour trees) on top of the saved map
- Taller sensor mast and a moving-obstacle scenario
- Real-robot bring-up

## License

Apache-2.0 (see `LICENSE`).