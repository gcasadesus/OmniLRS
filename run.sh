#!/bin/bash
cd /home/guillemc/dev/OmniLRS

# Deactivate conda if active
conda deactivate 2>/dev/null || true
conda deactivate 2>/dev/null || true

# Unset any conflicting variables
unset ROS_DISTRO
unset RMW_IMPLEMENTATION
# Force FastDDS to use UDP only (Avoid shared memory permission issues)
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/guillemc/dev/OmniLRS/fastdds_no_shm.xml

# Set up for humble explicitly (as per Isaac Sim error message)
export ROS_DOMAIN_ID=0
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=/home/guillemc/dev/isaac-sim-5.0.0/exts/isaacsim.ros2.bridge/humble/lib
export PYTHONPATH=/home/guillemc/dev/isaac-sim-5.0.0/exts/isaacsim.ros2.bridge/humble/rclpy

# Run Isaac Sim
# Pass through all arguments to allow overriding configs
# Add default streaming arguments if not present

# Set Public Endpoint to the machine's LAN IP to allow remote connection
IP_ADDR=$(hostname -I | awk '{print $1}')
ARGS="$@"
if [[ "$ARGS" != *"/app/livestream/publicEndpointAddress"* ]]; then
    ARGS="$ARGS --/app/livestream/publicEndpointAddress=$IP_ADDR"
fi
if [[ "$ARGS" != *"/app/livestream/port"* ]]; then
    ARGS="$ARGS --/app/livestream/port=49100"
fi

/home/guillemc/dev/isaac-sim-5.0.0/python.sh run.py $ARGS