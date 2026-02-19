#!/bin/bash
# Run the Mayara→ROS2 bridge
# Usage: ./run_bridge.sh [mayara_ws_url]
#
# Defaults to ws://10.0.0.28:6502/v2/api/radars/radar-11/spokes
# Override with: ./run_bridge.sh ws://localhost:6502/v2/api/radars/radar-11/spokes

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS2
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi

# Source any local workspace overlay (for marine_sensor_msgs)
if [ -f ~/echoflow_ws/install/setup.bash ]; then
    source ~/echoflow_ws/install/setup.bash
elif [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
elif [ -f ~/colcon_ws/install/setup.bash ]; then
    source ~/colcon_ws/install/setup.bash
fi

# Check for websocket-client
python3 -c "import websocket" 2>/dev/null || {
    echo "Installing websocket-client..."
    pip install websocket-client --break-system-packages
}

# Override WebSocket URL if provided
if [ -n "$1" ]; then
    export MAYARA_WS_URL="$1"
fi

echo "============================================================"
echo "  Mayara → ROS2 Bridge"
echo "============================================================"
echo "  Bridge dir: $SCRIPT_DIR"
echo "  ROS_DISTRO: $ROS_DISTRO"
echo ""
echo "  Ctrl+C to stop"
echo "============================================================"

# Add our module to Python path and run
export PYTHONPATH="$SCRIPT_DIR:$PYTHONPATH"
cd "$SCRIPT_DIR"
python3 bridge_node.py "$@"
