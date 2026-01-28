#!/bin/bash
# Script to run simulation with motor control
# Starts Gazebo simulation and ROS2 motor controller

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WORLD_FILE="$SCRIPT_DIR/ar_robot_water_world_hydrodynamics.sdf"

echo "=========================================="
echo "  AR Robot Simulation with Motors"
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

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "WARNING: ROS2 not sourced. Attempting to source..."
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source /opt/ros/jazzy/setup.bash
    else
        echo "ERROR: ROS2 not found. Please source ROS2 setup.bash"
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
echo "Starting ROS2 motor controller..."
echo ""

# Source the package if built
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
fi

# Run motor controller
python3 "$SCRIPT_DIR/motor_controller.py" &
CONTROLLER_PID=$!

echo ""
echo "=========================================="
echo "  Simulation Running"
echo "=========================================="
echo "Gazebo PID: $SIM_PID"
echo "Controller PID: $CONTROLLER_PID"
echo ""
echo "To control motors, use:"
echo "  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}'"
echo ""
echo "Or run test script:"
echo "  python3 ar_robot/test_motor_control.py"
echo ""
echo "Press Ctrl+C to stop..."
echo ""

# Wait for user interrupt
trap "kill $SIM_PID $CONTROLLER_PID 2>/dev/null; exit" INT TERM
wait
