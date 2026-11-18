#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "$SCRIPT_DIR/sim_env.sh"

echo "===================================="
echo "  LTMANG - MECANUM KEYBOARD TELEOP"
echo "===================================="

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"

if [[ "${ROS_DOMAIN_ID:-}" != "31" ||
      "${ROS_LOCALHOST_ONLY:-}" != "1" ]]; then
    echo "ERROR: Sai moi truong ROS simulation."
    exit 1
fi

# Khong cho hai nguon dieu khien tranh nhau /cmd_vel.
if ros2 node list 2>/dev/null |
   grep -Eq '^/(controller_server|velocity_smoother)$'; then
    echo "ERROR: Nav2 Navigation van dang chay."
    echo "Hay dung run_navigation.sh truoc khi dieu khien tay."
    exit 1
fi

echo ""
echo "Khoi dong keyboard teleop..."
echo ""

exec ros2 run keyboard_teleop keyboard_teleop_node \
  --ros-args \
  -p use_sim_time:=true \
  -p angular_speed:=0.2 \
  -p strafe_speed:=0.06 \
  -p slow_speed:=0.03 \
  -p normal_speed:=0.06 \
  -p fast_speed:=0.08 \
  -p publish_rate_hz:=10.0

