#!/usr/bin/env bash
# Bring up the SAR demo stack: agent -> SITL -> ROS nodes.
# Usage: scripts/stack_up.sh [launch-file] [gui]
#   default launch file: sar_bringup mission.launch.py
#   gui: render on Xvfb :99 (llvmpipe GLX) + serve via x11vnc on :5900,
#        starts Gazebo GUI; use with demo.launch.py rviz:=true for RViz too
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
LAUNCH="${1:-sar_bringup mission.launch.py}"
GUI="${2:-}"
LOG=/tmp/opencode

# headless EGL software rendering (see docs/research.md section 0.10)
GZ_ENV="env -u DISPLAY HEADLESS=1 LIBGL_ALWAYS_SOFTWARE=1 __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/50_mesa.json"

if [ "$GUI" = "gui" ]; then
  if ! pgrep -x Xvfb >/dev/null; then
    echo "starting Xvfb :99"
    Xvfb :99 -screen 0 2560x1000x24 > "$LOG/xvfb.log" 2>&1 &
    sleep 2
  fi
  if ! pgrep -x openbox >/dev/null; then
    DISPLAY=:99 openbox > "$LOG/openbox.log" 2>&1 &
    sleep 1
  fi
  if ! pgrep -x x11vnc >/dev/null; then
    echo "starting x11vnc on :5900 (macOS Screen Sharing prompts for it: password is 'sar-demo')"
    x11vnc -display :99 -forever -shared -passwd sar-demo -rfbport 5900 > "$LOG/x11vnc.log" 2>&1 &
  fi
  GZ_ENV="env DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1"
fi

echo "[1/3] starting MicroXRCEAgent (tmux:agent)"
tmux new-session -d -s agent "MicroXRCEAgent udp4 -p 8888 > $LOG/agent.log 2>&1"
sleep 2

echo "[2/3] starting PX4 SITL gz_x500_sar in sar_zone (tmux:sitl)"
tmux new-session -d -s sitl "source /opt/ros/lyrical/setup.bash; cd $PX4_DIR && $GZ_ENV PX4_GZ_WORLD=sar_zone make px4_sitl gz_x500_sar > $LOG/sitl.log 2>&1"

echo "  waiting for Gazebo world..."
for i in $(seq 1 40); do
  grep -aq "Gazebo world is ready" "$LOG/sitl.log" 2>/dev/null && break
  sleep 2
done
grep -aq "Gazebo world is ready" "$LOG/sitl.log" || { echo "SITL failed to start"; exit 1; }

echo "  waiting for XRCE-DDS session..."
for i in $(seq 1 20); do
  grep -aq "session established" "$LOG/agent.log" 2>/dev/null && break
  sleep 2
done

echo "[3/3] starting ROS nodes: ros2 launch $LAUNCH (tmux:mission)"
MISSION_ENV=""
if [ "$GUI" = "gui" ]; then
  MISSION_ENV="export DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1;"
fi
tmux new-session -d -s mission "$MISSION_ENV source /opt/ros/lyrical/setup.bash && source $HOME/ws_px4/install/setup.bash && source $REPO_DIR/install/setup.bash && ros2 launch $LAUNCH > $LOG/mission.log 2>&1"
echo "stack up. logs in $LOG/{agent,sitl,mission}.log"
