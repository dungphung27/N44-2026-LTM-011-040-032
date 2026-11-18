#!/bin/bash

# ROS 2 Humble
source /opt/ros/humble/setup.bash

# Workspace robot Mecanum
source "$HOME/mecanum_viz_ws/install/setup.bash"

# ROS Domain dành riêng cho mô phỏng
# Mini PC robot thật sử dụng Domain 30
export ROS_DOMAIN_ID=31

# Chỉ giao tiếp ROS 2 nội bộ PC
export ROS_LOCALHOST_ONLY=1

# Đường dẫn mô hình Gazebo
export GAZEBO_MODEL_PATH="$HOME/mecanum_viz_ws/src${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"

# Đường dẫn plugin quay bánh Mecanum
export GAZEBO_PLUGIN_PATH="$(ros2 pkg prefix mecanum_visual_animator)/lib${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
