#!/usr/bin/env bash
# Source me from EACH terminal. Build simulation/ros2_ws first.
_LTMANG_SIM_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ ! -f "$_LTMANG_SIM_DIR/ros2_ws/install/setup.bash" ]]; then
  echo "Missing built overlay: cd $_LTMANG_SIM_DIR/ros2_ws && colcon build --symlink-install" >&2
  return 1
fi
source "$_LTMANG_SIM_DIR/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=31
export ROS_LOCALHOST_ONLY=1
export LTMANG_SIM_ROOT="$_LTMANG_SIM_DIR"
export GAZEBO_MODEL_PATH="$_LTMANG_SIM_DIR/models:$_LTMANG_SIM_DIR/ros2_ws/src${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
_LTMANG_PLUGIN_LIB="$(ros2 pkg prefix mecanum_visual_animator)/lib"
export GAZEBO_PLUGIN_PATH="$_LTMANG_PLUGIN_LIB${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
unset _LTMANG_PLUGIN_LIB _LTMANG_SIM_DIR
