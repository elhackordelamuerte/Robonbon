#!/usr/bin/env bash
# Builds the workspace, launches the ROS2 nodes, and shuts them down cleanly
# on SIGTERM so destroy_node() runs (the real arm gets a proper disconnect).
set -e
set -m  # each background job gets its own process group

source /opt/ros/jazzy/setup.bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash

pids=()
launch() {
  local pkg=$1 file=$2
  shift 2
  if [ -f "install/$pkg/share/$pkg/launch/$file" ]; then
    ros2 launch "$pkg" "$file" "$@" &
    pids+=($!)
  fi
}

# Ensure serial device nodes exist in /dev for real hardware access
for i in 0 1 2 3; do
  [ -e /dev/ttyACM$i ] || mknod /dev/ttyACM$i c 166 $i 2>/dev/null && chmod 666 /dev/ttyACM$i 2>/dev/null || true
  [ -e /dev/ttyUSB$i ] || mknod /dev/ttyUSB$i c 188 $i 2>/dev/null && chmod 666 /dev/ttyUSB$i 2>/dev/null || true
done

# Real arm through a TCP bridge (Docker Desktop on macOS has no USB passthrough):
# scripts/serial_bridge.py relays the USB port on the host, socat recreates a serial port here.
if [ "${USE_SIM:-true}" = "false" ] && [ -n "${SERIAL_BRIDGE:-}" ]; then
  BRIDGE_PORT=/dev/ttyBRIDGE
  ( while true; do
      socat "pty,link=$BRIDGE_PORT,raw,echo=0" "$SERIAL_BRIDGE" \
        || echo "serial bridge $SERIAL_BRIDGE unreachable: is scripts/serial_bridge.py running on the host? retrying..."
      sleep 2
    done ) &
  for _ in $(seq 1 50); do [ -e "$BRIDGE_PORT" ] && break; sleep 0.1; done
  REAL_ROBOT_PORT=$BRIDGE_PORT
fi

launch so101_driver driver.launch.py "use_sim:=${USE_SIM:-true}" "port:=${REAL_ROBOT_PORT:-/dev/ttyACM0}"
# Real arm: never move by itself at startup (the brain's slow move to rest stays sim-only).
launch so101_brain brain.launch.py "rest_on_start:=${USE_SIM:-true}"

shutdown() {
  trap - TERM INT
  set +e  # wait returns the child status; errexit would abort the shutdown
  [ ${#pids[@]} -eq 0 ] && exit 0
  # Signal the whole group, as Ctrl-C does: ros2 launch ignores a SIGINT sent to it alone.
  for p in "${pids[@]}"; do kill -INT -"$p" 2>/dev/null; done
  ( sleep 5; for p in "${pids[@]}"; do kill -KILL -"$p" 2>/dev/null; done ) &
  for p in "${pids[@]}"; do wait "$p"; done
  exit 0
}

trap shutdown TERM INT

sleep infinity &
wait $!
