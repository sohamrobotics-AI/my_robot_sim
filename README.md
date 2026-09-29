cat > README.md <<'EOF'
# my_robot_sim

ROS 2 Humble + Gazebo Fortress simulation of a differential-drive robot with an RGB-D camera
and IMU. Wheel encoders and IMU are fused with visual odometry in an EKF (visual odometry
contributes yaw only). RTAB-Map builds a 2D map and a 3D point cloud.

## Requirements
Ubuntu 22.04, ROS 2 Humble

    sudo apt install ros-humble-desktop ros-humble-ros-gz ros-humble-xacro \
      ros-humble-robot-localization ros-humble-rtabmap-ros ros-humble-depth-image-proc \
      ros-humble-teleop-twist-keyboard ros-humble-nav2-map-server ros-humble-tf2-tools

## Build
    mkdir -p ~/ws/src && cd ~/ws/src
    git clone https://github.com/<sohamrobotics-AI>/my_robot_sim.git
    cd ~/ws && colcon build --symlink-install && source install/setup.bash

## Run
    ros2 launch my_robot_sim sim.launch.py          # terminal 1: Gazebo + robot + EKF
    ros2 launch my_robot_sim mapping.launch.py      # terminal 2: visual SLAM + RViz
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.5

Save the map:

    ros2 run nav2_map_server map_saver_cli -f ~/maps/retail_store

Localize in a saved map: `ros2 launch my_robot_sim mapping.launch.py localization:=true`

## Notes
- `sim.launch.py stereo:=true` adds an IR stereo pair; `launch/isaac_vslam.launch.py` swaps in
  Isaac ROS Visual SLAM (needs an NVIDIA GPU, untested).
- Options: `headless:=true`, `pointcloud:=false`, `rviz:=false`.
EOF