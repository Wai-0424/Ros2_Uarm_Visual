#!/usr/bin/env bash
# Wait for a serial device to appear, set permissions, then start the swiftpro_read_node_ros2
# Usage: ./wait_and_run_read.sh [DEVICE] [BAUD]

DEVICE=${1:-/dev/ttyACM0}
BAUD=${2:-115200}
LOGFILE="/tmp/swiftpro_read_node.log"

echo "[wait_and_run_read] waiting for device: $DEVICE" | tee -a "$LOGFILE"
while [ ! -e "$DEVICE" ]; do
  sleep 1
done

echo "[wait_and_run_read] device appeared: $DEVICE" | tee -a "$LOGFILE"

# try to set permissions (may require sudo)
if [ -w "$DEVICE" ]; then
  chmod 666 "$DEVICE" 2>/dev/null || true
else
  sudo chmod 666 "$DEVICE" 2>/dev/null || echo "[wait_and_run_read] failed to chmod, try running script with sudo" | tee -a "$LOGFILE"
fi

cd "$(dirname "$0")/.." || exit 1
echo "[wait_and_run_read] sourced workspace and launching node" | tee -a "$LOGFILE"
source install/setup.bash 2>/dev/null || true

# prefer installed binary, fallback to build artifact
if [ -x "install/swiftpro/lib/swiftpro/swiftpro_read_node_ros2" ]; then
  exec install/swiftpro/lib/swiftpro/swiftpro_read_node_ros2 >> "$LOGFILE" 2>&1
else
  # try ros2 run
  exec ros2 run swiftpro swiftpro_read_node_ros2 >> "$LOGFILE" 2>&1
fi
