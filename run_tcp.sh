#!/bin/bash

# Usage: ./run_tcp.sh [display|headless] [environment]
# Examples:
#   ./run_tcp.sh                          # headless, lunalab (default)
#   ./run_tcp.sh display                  # with display, lunalab
#   ./run_tcp.sh headless lunaryard_40m   # headless, lunaryard_40m
#   ./run_tcp.sh display largescale       # with display, largescale

# Parse arguments
MODE=${1:-headless}
ENVIRONMENT=${2:-lunalab}

# Determine headless flag
if [ "$MODE" = "display" ]; then
    HEADLESS="False"
    DISPLAY_ENABLED="true"
    echo "Running in DISPLAY mode with environment: $ENVIRONMENT"
else
    HEADLESS="True"
    DISPLAY_ENABLED="false"
    echo "Running in HEADLESS mode with environment: $ENVIRONMENT"
fi

# Ensure X server access (for display mode)
if [ "$DISPLAY_ENABLED" = "true" ]; then
    xhost + > /dev/null 2>&1
fi

# Stop any existing container
echo "Cleaning up existing container..."
docker stop isaac-sim-omnilrs-container 2>/dev/null || true
docker rm -f isaac-sim-omnilrs-container 2>/dev/null || true

# Wait for container to be fully removed
while docker ps -a | grep -q isaac-sim-omnilrs-container; do
    echo "Waiting for container to be removed..."
    sleep 1
    docker rm -f isaac-sim-omnilrs-container 2>/dev/null || true
done


# Common Docker arguments
DOCKER_ARGS=(
    --name isaac-sim-omnilrs-container
    --gpus all
    -e "ACCEPT_EULA=Y"
    -e "PRIVACY_CONSENT=Y"
    --rm
    --network=host
    --ipc=host
    -v "${PWD}:/workspace/omnilrs"
    -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw
    -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw
    -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw
    -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw
    -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw
    -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw
    -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw
    -v ~/docker/isaac-sim/documents:/root/Documents:rw
    -v /tmp/images_streaming:/tmp/images_streaming:rw
    -v /tmp/images_oncommand:/tmp/images_oncommand:rw
    -v /tmp/images_apxs:/tmp/images_apxs:rw
    -v /tmp/images_depth:/tmp/images_depth:rw
    -v /tmp/images_monitoring:/tmp/images_monitoring:rw
    -v /tmp/images_lander:/tmp/images_lander:rw
    -v bash_command_history:/commandhistory
)

# Add display support if needed
if [ "$DISPLAY_ENABLED" = "true" ]; then
    DOCKER_ARGS+=(-v "$HOME/.Xauthority:/root/.Xauthority" -e DISPLAY)
fi

# Run container
docker run -d "${DOCKER_ARGS[@]}" isaac-sim-omnilrs:latest \
  /bin/bash -c "cd /workspace/omnilrs && /isaac-sim/python.sh run.py environment=${ENVIRONMENT} mode=TCP rendering.renderer.headless=${HEADLESS}"

echo ""
echo "Container started. Following logs..."
echo "Press Ctrl+C to stop following logs (container will keep running)"
echo ""
echo "TCP Server will be available on: 127.0.0.1:5555"
echo ""
echo "Useful commands:"
echo "  Stop following logs: Ctrl+C"
echo "  Stop the container: docker stop isaac-sim-omnilrs-container"
echo "  Resume following logs: docker logs -f isaac-sim-omnilrs-container"
echo "  Connect with client: python clients/tcp_client.py"
echo ""
echo "----------------------------------------"
echo ""

# Follow the logs in real-time
docker logs -f isaac-sim-omnilrs-container
