#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
echo "Sourced ROS 2 ${ROS_DISTRO}"

if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
    echo "Sourced /ws workspace"
fi
exec "$@"