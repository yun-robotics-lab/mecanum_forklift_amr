#!/bin/bash
# Development safety net (not installed): stop the whole simulation when the CPU package runs hot.
# The development machine powered off twice under the full simulation (critical limit 100 C).
#   thermal_guard.sh [avg_limit_celsius] [poll_seconds] [window]    default 90 C, 2 s, 10 readings
# This CPU spikes by 20-25 C for a few seconds whenever a process starts (turbo burst), so single
# readings are meaningless: the simulation is stopped when the moving average over `window`
# readings reaches the limit, or at once when one reading reaches 99 C.
# Readings go to $LOG_DIR/thermal.log (default ~/.cache/mecanum_logs). Run it in the background
# next to a multi-robot simulation:  nohup thermal_guard.sh 90 &
LIMIT=${1:-90}
POLL=${2:-2}
WINDOW=${3:-10}
LOG_DIR=${LOG_DIR:-$HOME/.cache/mecanum_logs}
STACK="$(dirname "$(readlink -f "$0")")/../../mecanum_forklift_docking/scripts/dev_stack.sh"
mkdir -p "$LOG_DIR"
readings=()
while true; do
  temp=$(sensors 2>/dev/null | awk '/Package id 0/ {gsub(/[^0-9.]/, "", $4); print int($4); exit}')
  if [ -n "$temp" ]; then
    readings+=("$temp")
    [ "${#readings[@]}" -gt "$WINDOW" ] && readings=("${readings[@]:1}")
    sum=0
    for r in "${readings[@]}"; do sum=$((sum + r)); done
    avg=$((sum / ${#readings[@]}))
    echo "$(date +%T) ${temp} avg${avg}" >> "$LOG_DIR/thermal.log"
    if { [ "${#readings[@]}" -ge "$WINDOW" ] && [ "$avg" -ge "$LIMIT" ]; } || [ "$temp" -ge 99 ]; then
      echo "$(date +%T) THERMAL LIMIT: reading ${temp} C, ${WINDOW}-reading average ${avg} C" \
        "(limit ${LIMIT} C): stopping the simulation" | tee -a "$LOG_DIR/thermal.log"
      "$STACK" stop
      exit 1
    fi
  fi
  sleep "$POLL"
done
