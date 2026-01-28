#!/bin/bash
# Complete simulation script with motors
# Starts Gazebo, motor controller, and provides control interface

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WORLD_FILE="$SCRIPT_DIR/ar_robot_water_world_hydrodynamics.sdf"

echo "=========================================="
echo "  AR Robot Complete Simulation"
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
echo "To control motors, run in another terminal:"
echo "  python3 ar_robot/motor_controller_standalone.py"
echo ""
echo "Or use ROS2 (if available):"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  python3 ar_robot/motor_controller.py"
echo ""
echo "Press Ctrl+C to stop simulation..."
echo ""

# Wait for user interrupt
trap "kill $SIM_PID 2>/dev/null; exit" INT TERM
wait
