#!/usr/bin/env bash
set -euo pipefail
_here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$_here/sim_env.sh"
ros2 launch "$LTMANG_SIM_ROOT/launch/mecanum_gazebo_corridor_repo.launch.py" use_relay:=true
