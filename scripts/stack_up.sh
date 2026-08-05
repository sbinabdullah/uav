#!/usr/bin/env bash
# Bring up the SAR demo stack: agent -> SITL -> ROS nodes.
set -Eeuo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
LOG_DIR="${STACK_LOG_DIR:-/tmp/opencode}"
STATE_DIR="${STACK_STATE_DIR:-$LOG_DIR/sar-stack}"
LAUNCH_PACKAGE="sar_bringup"
LAUNCH_FILE="mission.launch.py"
LAUNCH_ARGS=()
GUI=0
START_VNC=1
DISPLAY_NUM="${STACK_DISPLAY:-99}"
VNC_PORT="${STACK_VNC_PORT:-5900}"
VNC_PASSWORD="${STACK_VNC_PASSWORD:-sar-demo}"
XRCE_PORT="${STACK_XRCE_PORT:-8888}"
RESTART=0

usage() {
  cat <<'EOF'
Usage: scripts/stack_up.sh [options] [launch-package launch-file [launch-args...]]

Options:
  --launch PACKAGE FILE  ROS 2 launch package and file
  --gui                  Run Gazebo/RViz on Xvfb and serve it with x11vnc
  --no-vnc               Start the GUI display without x11vnc
  --headless             Use headless software rendering (default)
  --display NUM          X display number (default: 99)
  --vnc-port PORT        x11vnc port (default: 5900)
  --vnc-password PASS    x11vnc password (default: sar-demo)
  --xrce-port PORT       MicroXRCEAgent UDP port (default: 8888)
  --px4-dir DIR          PX4-Autopilot directory
  --log-dir DIR          Log directory (default: /tmp/opencode)
  --restart              Stop this stack before starting it
  -h, --help             Show this help

The old form "sar_bringup demo.launch.py" is also accepted as the first
argument. Put ROS launch arguments after the package and launch file, for
example: --launch sar_bringup demo.launch.py rviz:=true
EOF
}

die() {
  echo "stack_up: $*" >&2
  exit 1
}

need_value() {
  (($# >= 2)) || die "$1 requires a value"
}

normalize_display() {
  DISPLAY_NUM="${DISPLAY_NUM#:}"
  [[ "$DISPLAY_NUM" =~ ^[0-9]+$ ]] || die "display must be a number: $DISPLAY_NUM"
}

valid_number() {
  [[ "$1" =~ ^[0-9]+$ ]]
}

has_session() {
  tmux has-session -t "$1" 2>/dev/null
}

pid_matches() {
  local pid="$1"
  local expected="$2"
  local command_line
  [[ "$pid" =~ ^[0-9]+$ ]] || return 1
  [[ -r "/proc/$pid/cmdline" ]] || return 1
  command_line="$(tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null || true)"
  [[ "$command_line" == *"$expected"* ]]
}

quote() {
  printf '%q' "$1"
}

start_tmux() {
  local session="$1"
  local command="$2"
  if has_session "$session"; then
    die "tmux:$session already exists; use --restart or scripts/stack_down.sh"
  fi
  tmux new-session -d -s "$session" "$command"
}

wait_for_log() {
  local file="$1"
  local pattern="$2"
  local attempts="$3"
  local i
  for ((i = 1; i <= attempts; i++)); do
    if [[ -f "$file" ]] && grep -aq "$pattern" "$file"; then
      return 0
    fi
    sleep 2
  done
  return 1
}

start_gui() {
  local display=":$DISPLAY_NUM"
  local xvfb_pid openbox_pid vnc_pid
  mkdir -p "$STATE_DIR"

  if ! xdpyinfo -display "$display" >/dev/null 2>&1; then
    echo "starting Xvfb $display"
    Xvfb "$display" -screen 0 2560x1000x24 >"$LOG_DIR/xvfb.log" 2>&1 &
    xvfb_pid=$!
    printf '%s\n' "$xvfb_pid" >"$STATE_DIR/xvfb.pid"
    for _ in {1..20}; do
      xdpyinfo -display "$display" >/dev/null 2>&1 && break
      sleep 0.25
    done
    xdpyinfo -display "$display" >/dev/null 2>&1 || die "Xvfb failed to start; see $LOG_DIR/xvfb.log"
  else
    echo "using existing X display $display"
  fi

  if [[ ! -s "$STATE_DIR/openbox.pid" ]] || ! pid_matches "$(<"$STATE_DIR/openbox.pid")" openbox; then
    echo "starting openbox on $display"
    DISPLAY="$display" openbox >"$LOG_DIR/openbox.log" 2>&1 &
    openbox_pid=$!
    printf '%s\n' "$openbox_pid" >"$STATE_DIR/openbox.pid"
  else
    echo "openbox is already running"
  fi

  if ((START_VNC)); then
    if [[ -s "$STATE_DIR/x11vnc.pid" ]] && pid_matches "$(<"$STATE_DIR/x11vnc.pid")" x11vnc; then
      echo "x11vnc is already running on port $VNC_PORT"
    else
      command -v x11vnc >/dev/null 2>&1 || die "x11vnc is required for --gui (use --no-vnc to skip it)"
      local password_file="$STATE_DIR/x11vnc.pass"
      umask 077
      x11vnc -storepasswd "$VNC_PASSWORD" "$password_file" >/dev/null
      echo "starting x11vnc on $display port $VNC_PORT"
      DISPLAY="$display" x11vnc -display "$display" -forever -shared \
        -rfbauth "$password_file" -rfbport "$VNC_PORT" >"$LOG_DIR/x11vnc.log" 2>&1 &
      vnc_pid=$!
      printf '%s\n' "$vnc_pid" >"$STATE_DIR/x11vnc.pid"
    fi
  fi
}

cleanup_on_error() {
  local rc=$?
  trap - EXIT
  if ((rc != 0)); then
    echo "stack_up: startup failed; cleaning up partial stack" >&2
    "$REPO_DIR/scripts/stack_down.sh" --log-dir "$LOG_DIR" --display "$DISPLAY_NUM" --quiet || true
  fi
  exit "$rc"
}

while (($#)); do
  case "$1" in
    --launch)
      (($# >= 3)) || die "--launch requires PACKAGE and FILE"
      LAUNCH_PACKAGE="$2"
      LAUNCH_FILE="$3"
      shift 3
      LAUNCH_ARGS+=("$@")
      break
      ;;
    --gui)
      GUI=1
      shift
      ;;
    --no-vnc)
      GUI=1
      START_VNC=0
      shift
      ;;
    --headless)
      GUI=0
      shift
      ;;
    --display)
      need_value "$@"
      DISPLAY_NUM="$2"
      shift 2
      ;;
    --vnc-port)
      need_value "$@"
      VNC_PORT="$2"
      shift 2
      ;;
    --vnc-password)
      need_value "$@"
      VNC_PASSWORD="$2"
      shift 2
      ;;
    --xrce-port)
      need_value "$@"
      XRCE_PORT="$2"
      shift 2
      ;;
    --px4-dir)
      need_value "$@"
      PX4_DIR="$2"
      shift 2
      ;;
    --log-dir)
      need_value "$@"
      LOG_DIR="$2"
      STATE_DIR="${STACK_STATE_DIR:-$LOG_DIR/sar-stack}"
      shift 2
      ;;
    --restart)
      RESTART=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      LAUNCH_ARGS+=("$@")
      break
      ;;
    -* )
      die "unknown option: $1"
      ;;
    *)
      if [[ "$1" == *" "* ]]; then
        read -r LAUNCH_PACKAGE LAUNCH_FILE <<<"$1"
        shift
      else
        LAUNCH_PACKAGE="$1"
        shift
        (($#)) || die "launch package requires a launch file"
        LAUNCH_FILE="$1"
        shift
      fi
      LAUNCH_ARGS+=("$@")
      break
      ;;
  esac
done

normalize_display
valid_number "$VNC_PORT" || die "VNC port must be a number: $VNC_PORT"
valid_number "$XRCE_PORT" || die "XRCE port must be a number: $XRCE_PORT"
command -v tmux >/dev/null 2>&1 || die "tmux is required"
command -v MicroXRCEAgent >/dev/null 2>&1 || die "MicroXRCEAgent is required"
[[ -d "$PX4_DIR" ]] || die "PX4 directory does not exist: $PX4_DIR"
[[ -f /opt/ros/lyrical/setup.bash ]] || die "ROS setup not found: /opt/ros/lyrical/setup.bash"
[[ -f "$HOME/ws_px4/install/setup.bash" ]] || die "PX4 ROS workspace setup not found: $HOME/ws_px4/install/setup.bash"
[[ -f "$REPO_DIR/install/setup.bash" ]] || die "repository setup not found; run colcon build first"

mkdir -p "$LOG_DIR" "$STATE_DIR"

if ((RESTART)); then
  "$REPO_DIR/scripts/stack_down.sh" --log-dir "$LOG_DIR" --display "$DISPLAY_NUM" --quiet
fi

for session in mission bridge offboard sitl agent; do
  has_session "$session" && die "tmux:$session already exists; use --restart or scripts/stack_down.sh"
done

trap cleanup_on_error EXIT

if ((GUI)); then
  command -v Xvfb >/dev/null 2>&1 || die "Xvfb is required for --gui"
  command -v openbox >/dev/null 2>&1 || die "openbox is required for --gui"
  command -v xdpyinfo >/dev/null 2>&1 || die "xdpyinfo is required for --gui"
  start_gui
  RENDER_ENV="export DISPLAY=$(quote ":$DISPLAY_NUM"); export LIBGL_ALWAYS_SOFTWARE=1;"
else
  RENDER_ENV="unset DISPLAY; export HEADLESS=1 LIBGL_ALWAYS_SOFTWARE=1 __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/50_mesa.json;"
fi

AGENT_CMD="exec MicroXRCEAgent udp4 -p $(quote "$XRCE_PORT") >$(quote "$LOG_DIR/agent.log") 2>&1"
echo "[1/3] starting MicroXRCEAgent (tmux:agent)"
start_tmux agent "$AGENT_CMD"
sleep 2

SITL_CMD="source $(quote /opt/ros/lyrical/setup.bash); $RENDER_ENV cd $(quote "$PX4_DIR") && PX4_GZ_WORLD=sar_zone make px4_sitl gz_x500_sar >$(quote "$LOG_DIR/sitl.log") 2>&1"
echo "[2/3] starting PX4 SITL gz_x500_sar in sar_zone (tmux:sitl)"
start_tmux sitl "$SITL_CMD"

echo "  waiting for Gazebo world..."
wait_for_log "$LOG_DIR/sitl.log" "Gazebo world is ready" 40 || die "SITL failed to start; see $LOG_DIR/sitl.log"

echo "  waiting for XRCE-DDS session..."
wait_for_log "$LOG_DIR/agent.log" "session established" 20 || echo "  XRCE-DDS session not confirmed; continuing"

launch_cmd=(ros2 launch "$LAUNCH_PACKAGE" "$LAUNCH_FILE" "${LAUNCH_ARGS[@]}")
printf -v launch_cmd_str '%q ' "${launch_cmd[@]}"
MISSION_ENV="$RENDER_ENV"
MISSION_CMD="$MISSION_ENV source $(quote /opt/ros/lyrical/setup.bash) && source $(quote "$HOME/ws_px4/install/setup.bash") && source $(quote "$REPO_DIR/install/setup.bash") && $launch_cmd_str >$(quote "$LOG_DIR/mission.log") 2>&1"
echo "[3/3] starting ROS nodes: ${launch_cmd[*]} (tmux:mission)"
start_tmux mission "$MISSION_CMD"
trap - EXIT
echo "stack up. logs in $LOG_DIR/{agent,sitl,mission}.log"
