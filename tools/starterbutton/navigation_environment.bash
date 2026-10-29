#!/bin/bash
# Explicit R1 navigation underlay. No ~/.rpprc or mutable global_env.config.
# The caller still sources its own navigation install and checks package origins.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH
export PATH=/usr/bin:/bin:/opt/ros/humble/bin:/usr/local/cuda-12.8/bin
export LD_LIBRARY_PATH=/usr/local/lib:/usr/local/cuda-12.8/lib64
source /opt/ros/humble/setup.bash || return 1
source /home/rpp/rpp_ws/install/local_setup.bash || return 1
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
