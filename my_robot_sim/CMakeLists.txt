cmake_minimum_required(VERSION 3.8)
project(my_robot_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch config worlds models rviz
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/sim_covariance_fix.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
