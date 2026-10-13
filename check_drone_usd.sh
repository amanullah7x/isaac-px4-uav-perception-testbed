#!/bin/bash
# Run drone_usd_check.py with Isaac Sim's bundled USD (pxr) libraries, without starting Isaac Sim or touching the GPU.
# Usage: ./check_drone_usd.sh /path/to/my_drone.usd
ISAACSIM_PATH="${ISAACSIM_PATH:-$HOME/isaacsim}"
USDLIBS=$(ls -d "$ISAACSIM_PATH"/extscache/omni.usd.libs-* 2>/dev/null | head -n1)
PHYSX_SCHEMA=$(ls -d "$ISAACSIM_PATH"/extscache/omni.usd.schema.physx-* 2>/dev/null | head -n1)
if [ -z "$USDLIBS" ]; then
    echo "Could not find omni.usd.libs under $ISAACSIM_PATH/extscache" >&2
    exit 2
fi
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHONPATH="$USDLIBS:$PHYSX_SCHEMA" \
LD_LIBRARY_PATH="$USDLIBS/bin:$PHYSX_SCHEMA/bin:$LD_LIBRARY_PATH" \
    "$ISAACSIM_PATH/kit/python/bin/python3" "$HERE/drone_usd_check.py" "$@" 2> >(grep -v "Could not open asset" >&2)
