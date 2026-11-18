#!/usr/bin/env bash
set -eo pipefail
_here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$_here/sim_env.sh"
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true autostart:=true \
            params_file:="$LTMANG_SIM_ROOT/config/nav2_params_mecanum_sim.yaml"
