#!/bin/bash
# Script to run Gazebo simulation with AR Robot in water world

# Get the script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WORLD_FILE="$SCRIPT_DIR/ar_robot_water_world.sdf"

echo "=========================================="
echo "  AR Robot Gazebo Simulation"
echo "=========================================="
echo ""
echo "Project directory: $PROJECT_DIR"
echo "World file: $WORLD_FILE"
echo ""

# Check if world file exists
if [ ! -f "$WORLD_FILE" ]; then
    echo "ERROR: World file not found at $WORLD_FILE"
    exit 1
fi

# Check if gz is available
if ! command -v gz &> /dev/null; then
    echo "ERROR: Gazebo (gz) is not installed or not in PATH"
    echo "Please install Gazebo: https://gazebosim.org/docs"
    exit 1
fi

echo "Starting Gazebo simulation..."
echo "Press Ctrl+C to stop the simulation"
echo ""

# Change to project directory and run simulation
cd "$PROJECT_DIR"
gz sim "$WORLD_FILE"
