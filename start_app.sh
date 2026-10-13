#!/bin/bash
# Starts the drone launcher app (camera feed + detection, buttons for Isaac Sim / PX4 / QGroundControl).
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
exec "$HERE/venv/bin/python" "$HERE/app.py" "$@"
