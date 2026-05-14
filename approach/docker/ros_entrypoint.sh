#!/bin/bash
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ -f /home/nvidia/ros2_ws/install/setup.bash ]; then
  source /home/nvidia/ros2_ws/install/setup.bash
fi

exec "$@"
