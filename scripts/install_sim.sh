#!/usr/bin/env bash
# Install sar simulation assets (world + models) into the PX4-Autopilot tree.
# Idempotent. Usage: scripts/install_sim.sh [path-to-PX4-Autopilot]
set -euo pipefail

PX4_DIR="${1:-$HOME/PX4-Autopilot}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

GZ_MODELS="$PX4_DIR/Tools/simulation/gz/models"
GZ_WORLDS="$PX4_DIR/Tools/simulation/gz/worlds"

for d in "$GZ_MODELS" "$GZ_WORLDS"; do
  [ -d "$d" ] || { echo "error: $d not found (PX4 tree?)"; exit 1; }
done

echo "installing world -> $GZ_WORLDS/sar_zone.sdf"
cp "$REPO_DIR/sim/worlds/sar_zone.sdf" "$GZ_WORLDS/"

for m in "$REPO_DIR"/sim/models/*/; do
  name="$(basename "$m")"
  echo "installing model -> $GZ_MODELS/$name"
  rm -rf "${GZ_MODELS:?}/$name"
  cp -r "$m" "$GZ_MODELS/$name"
done

echo "done. Run: PX4_GZ_WORLD=sar_zone make px4_sitl gz_x500_mono_cam_down"
