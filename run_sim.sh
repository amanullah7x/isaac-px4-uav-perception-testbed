#!/bin/bash
# Launch launch_custom_drone.py with the environment Isaac Sim's ROS 2 bridge needs.
#   ./run_sim.sh --usd /path/to/my_drone.usd [--cam-mount down] [--rtsp-port 8554] ...
# Isaac Sim 6 runs Python 3.12, so it must use its OWN bundled ROS 2 Humble libraries, not /opt/ros/humble (Python 3.10).
# Do not `source /opt/ros/humble/setup.bash` in this terminal.
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ISAACSIM_PATH="${ISAACSIM_PATH:-$HOME/isaacsim}"
export ISAACSIM_PYTHON="${ISAACSIM_PYTHON:-$ISAACSIM_PATH/python.sh}"

if [ -n "$AMENT_PREFIX_PATH" ] || [[ "$PYTHONPATH" == */opt/ros/* ]]; then
    echo "[run_sim] /opt/ros is sourced in this shell; stripping it for Isaac Sim (Python 3.12)." >&2
    unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH ROS_PYTHON_VERSION ROS_VERSION ROS_LOCALHOST_ONLY
    export PYTHONPATH=$(echo "$PYTHONPATH" | tr ':' '\n' | grep -v "^/opt/ros" | paste -sd: -)
    export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v "^/opt/ros" | paste -sd: -)
fi
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$ISAACSIM_PATH/exts/isaacsim.ros2.core/humble/lib"

# Leftover PX4 instances from a crashed run keep TCP 4560 / UDP 14550 busy; warn instead of killing silently.
if pgrep -x px4 >/dev/null; then
    echo "[run_sim] WARNING: a px4 process is already running (pid $(pgrep -x px4 | tr '\n' ' '))." >&2
    echo "[run_sim]          If it is left over from a previous run: pkill -x px4" >&2
fi

exec "$ISAACSIM_PYTHON" "$HERE/launch_custom_drone.py" "$@"
