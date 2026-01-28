#!/bin/bash
# Script to view ar_robot.urdf in WSL

# Get the script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
URDF_FILE="$SCRIPT_DIR/ar_robot.urdf"

echo "=========================================="
echo "  AR Robot URDF Viewer (WSL)"
echo "=========================================="
echo ""
echo "Project directory: $PROJECT_DIR"
echo "URDF file: $URDF_FILE"
echo ""

# Check if URDF file exists
if [ ! -f "$URDF_FILE" ]; then
    echo "ERROR: URDF file not found at $URDF_FILE"
    exit 1
fi

# Check if check_urdf is available (from urdf_parser_py or urdfdom)
if command -v check_urdf &> /dev/null; then
    echo "Validating URDF..."
    echo "-------------------"
    if check_urdf "$URDF_FILE"; then
        echo "✓ URDF is valid!"
    else
        echo "✗ URDF validation failed!"
    fi
    echo ""
fi

# Display URDF content with syntax highlighting if available
echo "URDF Content:"
echo "-------------"
if command -v bat &> /dev/null; then
    bat "$URDF_FILE"
elif command -v highlight &> /dev/null; then
    highlight -O xterm256 "$URDF_FILE"
else
    cat "$URDF_FILE"
fi

echo ""
echo ""
echo "=========================================="
echo "  Visualization Options:"
echo "=========================================="
echo ""
echo "1. View in Gazebo (recommended):"
echo "   cd $PROJECT_DIR"
echo "   gz sim ar_robot/ar_robot_water_world.sdf"
echo ""
echo "2. Generate URDF tree graph:"
echo "   urdf_to_graphiz $URDF_FILE"
echo "   (then view the generated PDF)"
echo ""
echo "3. Convert URDF to SDF:"
echo "   gz sdf -p $URDF_FILE > ar_robot/model.sdf"
echo ""
echo "4. View with RViz (if ROS is installed):"
echo "   rosrun rviz rviz"
echo "   (then load the URDF in RViz)"
echo ""