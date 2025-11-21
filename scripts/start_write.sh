#!/usr/bin/env bash
set -euo pipefail
# Start the write node (with enable_writes default false) with workspace env

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

WRITE_OUT=/tmp/swiftpro_write_node.log
WRITE_PID=/tmp/swiftpro_write_node.pid
rm -f "$WRITE_OUT" "$WRITE_PID"

echo "Starting swiftpro_write_node_ros2 (enable_writes:=false)... logs: $WRITE_OUT"
nohup "$ROOT_DIR/install/swiftpro/lib/swiftpro/swiftpro_write_node_ros2" --ros-args -p enable_writes:=false > "$WRITE_OUT" 2>&1 &
echo $! > "$WRITE_PID"
sleep 1
echo "write pid: $(cat "$WRITE_PID")"
echo "tailing last 80 lines of $WRITE_OUT"
tail -n 80 "$WRITE_OUT" || true
