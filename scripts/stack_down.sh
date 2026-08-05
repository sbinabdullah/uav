#!/usr/bin/env bash
# Gracefully stop the SAR demo stack and optionally its Xvfb/x11vnc display.
set -Eeuo pipefail

LOG_DIR="${STACK_LOG_DIR:-/tmp/opencode}"
STATE_DIR="${STACK_STATE_DIR:-$LOG_DIR/sar-stack}"
DISPLAY_NUM="${STACK_DISPLAY:-99}"
VNC_PORT="${STACK_VNC_PORT:-5900}"
CLEAN_SHM=1
KEEP_DISPLAY=0
QUIET=0
FORCE=0

usage() {
  cat <<'EOF'
Usage: scripts/stack_down.sh [options]

Options:
  --log-dir DIR       Log/state directory (default: /tmp/opencode)
  --display NUM       X display number to stop (default: 99)
  --vnc-port PORT     Legacy x11vnc port for --force cleanup (default: 5900)
  --keep-display      Leave Xvfb, openbox, and x11vnc running
  --no-clean-shm      Do not remove /dev/shm/fastdds_* segments
  --force             Also stop a matching legacy x11vnc process without a PID file
  -q, --quiet         Suppress routine status messages
  -h, --help          Show this help
EOF
}

log() {
  ((QUIET)) || echo "$*"
}

die() {
  echo "stack_down: $*" >&2
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

stop_session() {
  local session="$1"
  if ! has_session "$session"; then
    return 0
  fi
  log "stopping tmux:$session"
  tmux send-keys -t "$session" C-c 2>/dev/null || true
  sleep 0.5
  tmux kill-session -t "$session" 2>/dev/null || true
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

stop_owned_process() {
  local name="$1"
  local expected="$2"
  local pid_file="$STATE_DIR/$name.pid"
  local pid
  if [[ ! -s "$pid_file" ]]; then
    return 0
  fi
  pid="$(<"$pid_file")"
  if ! pid_matches "$pid" "$expected"; then
    log "removing stale $name PID file"
    rm -f "$pid_file"
    return 0
  fi
  log "stopping $name (pid $pid)"
  kill -TERM "$pid" 2>/dev/null || true
  for _ in {1..20}; do
    kill -0 "$pid" 2>/dev/null || break
    sleep 0.25
  done
  if kill -0 "$pid" 2>/dev/null; then
    log "$name did not exit; sending SIGKILL"
    kill -KILL "$pid" 2>/dev/null || true
  fi
  rm -f "$pid_file"
}

while (($#)); do
  case "$1" in
    --log-dir)
      need_value "$@"
      LOG_DIR="$2"
      STATE_DIR="${STACK_STATE_DIR:-$LOG_DIR/sar-stack}"
      shift 2
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
    --keep-display)
      KEEP_DISPLAY=1
      shift
      ;;
    --no-clean-shm)
      CLEAN_SHM=0
      shift
      ;;
    --force)
      FORCE=1
      shift
      ;;
    -q|--quiet)
      QUIET=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown option: $1"
      ;;
  esac
done

normalize_display
valid_number "$VNC_PORT" || die "VNC port must be a number: $VNC_PORT"
mkdir -p "$STATE_DIR"

if command -v tmux >/dev/null 2>&1; then
  for session in mission offboard bridge sitl agent; do
    stop_session "$session"
  done
else
  log "tmux is not installed; skipping tmux sessions"
fi

if ((KEEP_DISPLAY == 0)); then
  stop_owned_process x11vnc x11vnc
  stop_owned_process openbox openbox
  stop_owned_process xvfb Xvfb
  rm -f "$STATE_DIR/x11vnc.pass"
fi

if ((FORCE && KEEP_DISPLAY == 0)); then
  if command -v pkill >/dev/null 2>&1; then
    if pkill -f "[x]11vnc.*-rfbport $VNC_PORT" 2>/dev/null; then
      log "stopped matching legacy x11vnc on port $VNC_PORT"
    fi
  fi
fi

if ((CLEAN_SHM)); then
  shopt -s nullglob
  fastdds_segments=(/dev/shm/fastdds_*)
  if ((${#fastdds_segments[@]})); then
    rm -f -- "${fastdds_segments[@]}"
    log "cleared /dev/shm/fastdds_*"
  else
    log "no Fast DDS shared-memory segments to clear"
  fi
fi

log "stack down"
