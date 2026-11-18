#!/usr/bin/env bash
set -euo pipefail
_here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$_here/sim_env.sh"
ros2 launch nav2_bringup localization_launch.py use_sim_time:=true autostart:=true \
            map:="$LTMANG_SIM_ROOT/maps/my_map.yaml" \
            params_file:="$LTMANG_SIM_ROOT/config/nav2_params_mecanum_sim.yaml"
