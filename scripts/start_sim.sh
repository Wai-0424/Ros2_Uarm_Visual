#!/usr/bin/env bash
set -euo pipefail
# Start the sim_publisher with workspace environment sourced so the background
# process has the correct PYTHONPATH. Writes PID to /tmp and logs to /tmp.

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT_DIR"

# Temporarily disable "nounset" when sourcing external setup scripts
# because those scripts may reference variables that are not set in this
# script's strict mode.
set +u
if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [ -f "$ROOT_DIR/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$ROOT_DIR/install/setup.bash"
fi
set -u

SIM_OUT=/tmp/swiftpro_sim.out
SIM_PID=/tmp/swiftpro_sim.pid
rm -f "$SIM_OUT" "$SIM_PID"

echo "Starting sim_publisher... logs: $SIM_OUT"
nohup python3 "$ROOT_DIR/install/swiftpro/lib/swiftpro/sim_publisher.py" > "$SIM_OUT" 2>&1 &
echo $! > "$SIM_PID"
sleep 1
echo "sim pid: $(cat "$SIM_PID")"
echo "tailing last 40 lines of $SIM_OUT"
tail -n 40 "$SIM_OUT" || true
