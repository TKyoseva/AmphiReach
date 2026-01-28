#!/bin/bash
# Script to check system versions: Ubuntu, Gazebo, ROS, and plugins

echo "=========================================="
echo "  System Version Information"
echo "=========================================="
echo ""

# Ubuntu Version
echo "--- Ubuntu Version ---"
if [ -f /etc/os-release ]; then
    source /etc/os-release
    echo "Distribution: $NAME"
    echo "Version: $VERSION"
    echo "Codename: $VERSION_CODENAME"
    echo "ID: $ID"
else
    echo "Cannot determine Ubuntu version"
fi
echo ""

# Linux Kernel
echo "--- Linux Kernel ---"
uname -r
echo ""

# Gazebo Version
echo "--- Gazebo (gz) Version ---"
if command -v gz &> /dev/null; then
    gz --version 2>&1 || echo "gz command found but version check failed"
else
    # Try alternative paths
    if [ -f "/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz" ]; then
        echo "Using: /opt/ros/jazzy/opt/gz_tools_vendor/bin/gz"
        /opt/ros/jazzy/opt/gz_tools_vendor/bin/gz --version 2>&1 || echo "Version check failed"
    else
        echo "gz command not found"
    fi
fi
echo ""

# Check for gz-sim specifically
echo "--- Gazebo Sim (gz sim) ---"
if command -v gz &> /dev/null; then
    gz sim --version 2>&1 || echo "gz sim not available or version check failed"
else
    echo "gz command not found"
fi
echo ""

# ROS Version
echo "--- ROS Version ---"
if [ -n "$ROS_DISTRO" ]; then
    echo "ROS_DISTRO: $ROS_DISTRO"
    if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
        echo "ROS Installation: /opt/ros/$ROS_DISTRO"
    fi
else
    echo "ROS_DISTRO not set (ROS may not be sourced)"
    
    # Check for installed ROS distributions
    echo "Checking for installed ROS distributions:"
    if [ -d "/opt/ros" ]; then
        ls -1 /opt/ros/ 2>/dev/null || echo "No ROS installations found in /opt/ros"
    else
        echo "No /opt/ros directory found"
    fi
fi
echo ""

# ROS2 Version (if available)
echo "--- ROS2 Version ---"
if command -v ros2 &> /dev/null; then
    ros2 --version 2>&1 || echo "ros2 command found but version check failed"
else
    echo "ros2 command not found"
fi
echo ""

# Python Version
echo "--- Python Version ---"
python3 --version 2>&1 || echo "python3 not found"
echo ""

# Check for Gazebo plugins
echo "--- Gazebo Plugins ---"
echo "Checking for common Gazebo plugins..."

# Check gz-sim plugins
if command -v gz &> /dev/null; then
    echo "Available gz-sim systems:"
    gz plugin --info 2>&1 | head -20 || echo "Could not list plugins"
else
    echo "gz command not available for plugin check"
fi
echo ""

# Check for specific plugins mentioned in URDF
echo "--- Required Plugins (from URDF) ---"
echo "Checking for plugins used in ar_robot.urdf:"
echo "  - gz-sim-hydrodynamics-system"
echo "  - gz-sim-joint-position-controller-system"
echo "  - gz-sim-user-commands-system"
echo ""

# Check installed packages
echo "--- Installed Gazebo Packages ---"
if command -v dpkg &> /dev/null; then
    echo "Gazebo-related packages:"
    dpkg -l | grep -i gazebo | grep -i "^ii" | awk '{print $2, $3}' || echo "No Gazebo packages found via dpkg"
elif command -v rpm &> /dev/null; then
    echo "Gazebo-related packages:"
    rpm -qa | grep -i gazebo || echo "No Gazebo packages found via rpm"
else
    echo "Package manager not found (not Debian/RedHat based)"
fi
echo ""

# Check ROS packages
echo "--- Installed ROS Packages ---"
if [ -n "$ROS_DISTRO" ]; then
    if command -v rosdep &> /dev/null; then
        echo "ROS packages (sample):"
        rospack list 2>/dev/null | head -10 || echo "Could not list ROS packages"
    fi
else
    echo "ROS not sourced, skipping ROS package check"
fi
echo ""

# Environment variables
echo "--- Relevant Environment Variables ---"
echo "GAZEBO_MODEL_PATH: ${GAZEBO_MODEL_PATH:-not set}"
echo "GAZEBO_RESOURCE_PATH: ${GAZEBO_RESOURCE_PATH:-not set}"
echo "IGN_GAZEBO_RESOURCE_PATH: ${IGN_GAZEBO_RESOURCE_PATH:-not set}"
echo "ROS_DISTRO: ${ROS_DISTRO:-not set}"
echo "ROS_VERSION: ${ROS_VERSION:-not set}"
echo ""

echo "=========================================="
echo "  Version Check Complete"
echo "=========================================="
