#!/usr/bin/env bash
# Helper to run robocupssl/ssl-game-controller in bridge mode (-p) and
# start a forwarder container that re-sends multicast referee packets to the host.
# Usage: ./scripts/run_controller_with_forwarder.sh [controller_args...]

set -euo pipefail

IMAGE=${IMAGE:-robocupssl/ssl-game-controller}
GC_NAME=${GC_NAME:-ssl-gc}
FORWARD_NAME=${FORWARD_NAME:-ssl-forward}
HOST_IF=${HOST_IF:-docker0}
MCAST_GRP=${MCAST_GRP:-224.5.23.1}
MCAST_PORT=${MCAST_PORT:-10003}
UI_PORT=${UI_PORT:-8081}

# extra args passed to controller
EXTRA_ARGS="$@"

echo "Using image: $IMAGE"
echo "Using image: $IMAGE"

# Remove old forwarder if it exists
if docker ps -a --format '{{.Names}}' | grep -q "^${FORWARD_NAME}$"; then
  echo "Removing old forwarder container ${FORWARD_NAME}"
  docker rm -f ${FORWARD_NAME} >/dev/null || true
fi

# If a container running from the same image already exists, reuse it
EXISTING=$(docker ps --filter ancestor=${IMAGE} --format '{{.Names}}' | head -n 1)
if [ -n "${EXISTING}" ]; then
  echo "Found existing game-controller container '${EXISTING}', reusing it (will not create a new container)."
  GC_NAME=${EXISTING}
else
  # If a container with the default name exists, remove it first
  if docker ps -a --format '{{.Names}}' | grep -q "^${GC_NAME}$"; then
    echo "Removing old game-controller container ${GC_NAME}"
    docker rm -f ${GC_NAME} >/dev/null || true
  fi

  # Run game-controller in bridge mode with ports mapped
  echo "Starting game-controller container (${GC_NAME}) mapping UI port ${UI_PORT} and referee port ${MCAST_PORT}"
  docker run -d --name ${GC_NAME} -p ${UI_PORT}:${UI_PORT} -p ${MCAST_PORT}:${MCAST_PORT} ${IMAGE} -address :${UI_PORT} ${EXTRA_ARGS}
fi

# Detect host IP on docker bridge (docker0)
HOST_IP=$(ip -4 addr show ${HOST_IF} 2>/dev/null | grep -oP 'inet \K[0-9.]+' || true)
if [ -z "$HOST_IP" ]; then
  echo "Could not detect host IP on interface ${HOST_IF}. Please set HOST_IP env var or adjust HOST_IF." >&2
  echo "Falling back to 172.17.0.1 (common default)." >&2
  HOST_IP=172.17.0.1
fi

echo "Detected host IP for containers: ${HOST_IP}"

# Start forwarder container sharing network namespace with game-controller
echo "Starting forwarder container (${FORWARD_NAME}) that will forward ${MCAST_GRP}:${MCAST_PORT} -> ${HOST_IP}:${MCAST_PORT}"

docker run -d --name ${FORWARD_NAME} --network container:${GC_NAME} python:3.11-slim bash -c "python - <<'PY'
import socket, struct, sys
MCAST_GRP = '${MCAST_GRP}'
MCAST_PORT = ${MCAST_PORT}
HOST_IP = '${HOST_IP}'

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('', MCAST_PORT))
mreq = struct.pack('4s4s', socket.inet_aton(MCAST_GRP), socket.inet_aton('0.0.0.0'))
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print('forwarder started', file=sys.stderr)
while True:
    data, addr = sock.recvfrom(65536)
    out.sendto(data, (HOST_IP, MCAST_PORT))
PY"

echo "All started."

echo "To run the referee node on the host pointing to forwarded port run (example):"
echo "  ros2 run referee referee_node --ros-args -p forward_port:=${MCAST_PORT} -p verbose:=false"

echo "To stop and clean up run:"
echo "  docker rm -f ${FORWARD_NAME} ${GC_NAME}"

exit 0
