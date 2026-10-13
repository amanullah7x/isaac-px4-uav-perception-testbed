#!/bin/bash
# Copy the Isaac Sim airframes (22000_isaac_iris, 22001_isaac_x500, 22002_isaac_fpv) into a PX4-Autopilot tree.
#   tools/install_px4_airframes.sh [~/PX4-Autopilot]
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4="${1:-$HOME/PX4-Autopilot}"
A="$PX4/ROMFS/px4fmu_common/init.d-posix/airframes"
cp "$HERE"/px4/airframes/2200*_isaac_* "$A/"
for f in 22000_isaac_iris 22001_isaac_x500 22002_isaac_fpv; do
    grep -q "^\s*$f$" "$A/CMakeLists.txt" || sed -i "s/^\(\s*\)10015_gazebo-classic_iris$/&\n\1$f/" "$A/CMakeLists.txt"
done
# Also into an existing SITL build, so no rebuild is needed (the startup script looks the airframe up at run time)
B="$PX4/build/px4_sitl_default/etc/init.d-posix/airframes"
[ -d "$B" ] && cp "$HERE"/px4/airframes/2200*_isaac_* "$B/"
echo "Installed Isaac airframes into $PX4"
