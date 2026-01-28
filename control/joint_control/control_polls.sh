#!/bin/bash
# Script to control poll rotation in Gazebo simulation
# Usage: ./control_polls.sh <poll_l_velocity> <poll_r_velocity>
# Example: ./control_polls.sh 1.0 -1.0  (rotate poll_l forward, poll_r backward)

POLL_L_VEL=${1:-0.0}
POLL_R_VEL=${2:-0.0}

echo "=========================================="
echo "  Poll Control Script"
echo "=========================================="
echo ""
echo "Poll L velocity: $POLL_L_VEL rad/s"
echo "Poll R velocity: $POLL_R_VEL rad/s"
echo ""
echo "To control polls in Gazebo:"
echo ""
echo "Method 1: Using Gazebo GUI"
echo "  1. In Gazebo, click on the robot model"
echo "  2. In the right panel, find 'Joints' tab"
echo "  3. Find 'base_link_Revolute-6' (poll_l) and 'base_link_Revolute-7' (poll_r)"
echo "  4. Adjust the velocity or position sliders"
echo ""
echo "Method 2: Using gz command line (if available)"
echo "  gz topic -t '/model/ar_robot/joint/base_link_Revolute-6/cmd' -m gz.msgs.Double -p 'data: $POLL_L_VEL'"
echo "  gz topic -t '/model/ar_robot/joint/base_link_Revolute-7/cmd' -m gz.msgs.Double -p 'data: $POLL_R_VEL'"
echo ""
echo "Method 3: Using gz service (if available)"
echo "  gz service -s /world/ar_robot_water_world/joint/cmd"
echo ""
echo "Joint Names:"
echo "  - base_link_Revolute-6: poll_l (left poll)"
echo "  - base_link_Revolute-7: poll_r (right poll)"
echo ""
echo "Note: Joints rotate around Z-axis (0.0 0.0 1.0)"
echo "Positive velocity = counter-clockwise (when viewed from above)"
echo "Negative velocity = clockwise"
echo ""
