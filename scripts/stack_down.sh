#!/usr/bin/env bash
# Stop all SAR demo processes and clean stale FastDDS shared-memory segments.
set -u
for s in mission bridge offboard sitl agent; do
  tmux kill-session -t "$s" 2>/dev/null && echo "killed tmux:$s"
done
pkill -x px4 2>/dev/null && echo "killed px4"
pkill -f "MicroXRCEAgen[t]" 2>/dev/null && echo "killed agent"
pkill -f "ros_gz_bridg[e]" 2>/dev/null && echo "killed bridge"
pkill -f "grid_mission_nod[e]" 2>/dev/null && echo "killed mission node"
pkill -f "gz-sim-mai[n]" 2>/dev/null && echo "killed gz-sim server"
sleep 1
rm -f /dev/shm/fastdds_* 2>/dev/null && echo "cleared /dev/shm/fastdds_*"
echo "stack down"
