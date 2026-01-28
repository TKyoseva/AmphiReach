#!/bin/bash
# Script to run mission simulation
# Starts Gazebo and mission controller

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WORLD_FILE="$SCRIPT_DIR/ar_robot_water_world_hydrodynamics.sdf"

echo "=========================================="
echo "  AR Robot Mission Simulation"
echo "=========================================="
echo ""

# Check if gz is available
if ! command -v gz &> /dev/null; then
    if [ -f "/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz" ]; then
        export PATH="/opt/ros/jazzy/opt/gz_tools_vendor/bin:$PATH"
    else
        echo "ERROR: gz command not found"
        exit 1
    fi
fi

cd "$PROJECT_DIR"

echo "Starting Gazebo simulation..."
gz sim "$WORLD_FILE" &
SIM_PID=$!

echo "Waiting for simulation to initialize..."
sleep 5

echo ""
echo "Simulation started (PID: $SIM_PID)"
echo ""
echo "To run mission:"
echo "  Option 1 (Standalone - No ROS2):"
echo "    python3 ar_robot/mission_controller_standalone.py"
echo ""
echo "  Option 2 (ROS2):"
echo "    source /opt/ros/jazzy/setup.bash"
echo "    python3 ar_robot/mission_controller.py"
echo ""
echo "Press Ctrl+C to stop simulation..."
echo ""

# Wait for user interrupt
trap "kill $SIM_PID 2>/dev/null; exit" INT TERM
wait
