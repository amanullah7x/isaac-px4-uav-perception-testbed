#!/bin/bash
# Download the scene assets from Gazebo Fuel and convert them to USD (Baylands park, red hatchback, standing person).
#   tools/fetch_assets.sh            needs: curl, unzip, python3 (creates tools/.venv with usd-core, trimesh, pycollada)
# Baylands: CC-BY 4.0 (OpenRobotics). Hatchback, Standing person: CC0 (OpenRobotics).
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$HERE/assets/src"; mkdir -p "$SRC"
PY="$HERE/tools/.venv/bin/python"
if [ ! -x "$PY" ]; then
    python3 -m venv "$HERE/tools/.venv"
    "$HERE/tools/.venv/bin/pip" install -q -r "$HERE/tools/requirements.txt"
fi
fetch() {  # fetch <Fuel model name> <folder>
    [ -d "$SRC/$2" ] && return
    curl -sL -o "$SRC/$2.zip" "https://fuel.gazebosim.org/1.0/OpenRobotics/models/${1// /%20}/tip/${1// /%20}.zip"
    mkdir -p "$SRC/$2" && unzip -o -q "$SRC/$2.zip" -d "$SRC/$2" && rm "$SRC/$2.zip"
}
echo "Downloading from Gazebo Fuel (Baylands is ~150 MB)..."
fetch "Baylands" baylands
fetch "Hatchback red" hatchback_red
fetch "Hatchback" hatchback
fetch "Standing person" standing_person

# Car: OBJ + its two textures side by side
T=$(mktemp -d)
cp "$SRC/hatchback_red/meshes/hatchback.obj" "$SRC/hatchback_red/meshes/hatchback.mtl" "$T/"
cp "$SRC/hatchback_red/materials/textures/hatchback.png" "$SRC/hatchback/materials/textures/wheels3.png" "$T/"
sed -i 's#../materials/textures/##' "$T/hatchback.mtl"
"$PY" "$HERE/tools/mesh_to_usd.py" "$T/hatchback.obj" "$HERE/assets/car/car.usd" --scale 0.0254 --rotz 90 --center --ground
# Person: DAE with textures next to it
cp "$SRC/standing_person/meshes/standing.dae" "$SRC"/standing_person/materials/textures/* "$T/"
sed -i 's#../materials/textures/##g' "$T/standing.dae"
"$PY" "$HERE/tools/mesh_to_usd.py" "$T/standing.dae" "$HERE/assets/person/person.usd" --center --ground
rm -rf "$T"
# Baylands: three DAE tiles, lowered 1.3 m as in its SDF, then ground colliders
"$PY" "$HERE/tools/mesh_to_usd.py" "$SRC"/baylands/media/baylands_0{1,2,3}.DAE "$HERE/assets/baylands/baylands.usd" --z-offset -1.3
"$PY" "$HERE/tools/add_colliders.py" "$HERE/assets/baylands/baylands.usd" "$SRC/baylands/media/Textures"
echo "Assets ready in $HERE/assets"
