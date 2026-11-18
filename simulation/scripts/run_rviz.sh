#!/usr/bin/env bash
set -euo pipefail
_here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$_here/sim_env.sh"
ros2 launch nav2_bringup rviz_launch.py use_sim_time:=true
