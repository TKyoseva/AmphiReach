# UUV Simulator Integration Guide

## Overview

UUV (Unmanned Underwater Vehicle) Simulator provides realistic underwater physics including:
- **Hydrodynamics**: Drag, lift, added mass effects
- **Buoyancy**: Realistic floating behavior
- **Thruster models**: Underwater propulsion
- **Water currents**: Environmental forces

## Important Notes

⚠️ **UUV Simulator was archived in February 2023** and is no longer actively maintained.

⚠️ **Compatibility Issues**:
- Designed for **Gazebo Classic** (not gz-sim/Gazebo Sim)
- Requires **ROS** (Robot Operating System)
- Your current setup uses **gz-sim** (`gz sim` command)

## Installation Options

### Option 1: Install UUV Simulator (Gazebo Classic + ROS)

If you want to use UUV Simulator, you'll need to:

1. **Install ROS** (if not already installed):
   ```bash
   # For Ubuntu 20.04 (ROS Noetic)
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
   sudo apt update
   sudo apt install ros-noetic-desktop-full
   ```

2. **Install Gazebo Classic**:
   ```bash
   sudo apt install gazebo11 libgazebo11-dev
   ```

3. **Install UUV Simulator**:
   ```bash
   sudo apt install ros-noetic-uuv-simulator
   # OR build from source:
   # https://github.com/uuvsimulator/uuv_simulator
   ```

4. **Convert your world to Gazebo Classic format** (SDF format is compatible)

### Option 2: Use gz-sim Hydrodynamics Plugin (Recommended)

Since you're using **gz-sim**, a better option is to use Gazebo Sim's built-in hydrodynamics:

1. **gz-sim-hydrodynamics plugin** - Provides buoyancy and drag forces
2. **gz-sim-buoyancy-engine plugin** - Realistic floating behavior

## Integration Steps

### For UUV Simulator (Gazebo Classic):

1. Convert your robot URDF to work with ROS
2. Add UUV plugins to your robot model
3. Use Gazebo Classic instead of gz-sim

### For gz-sim Hydrodynamics:

1. Add hydrodynamics plugin to your world file
2. Configure buoyancy parameters for your robot
3. Add drag coefficients

## Recommendation

**For your current setup (gz-sim)**, I recommend:
1. Using **gz-sim hydrodynamics plugins** (native to gz-sim)
2. OR creating a custom buoyancy plugin
3. OR switching to Gazebo Classic + UUV Simulator if you need advanced underwater physics

Would you like me to:
- **A)** Create a gz-sim hydrodynamics version of your world?
- **B)** Help convert to Gazebo Classic + UUV Simulator?
- **C)** Create a hybrid approach?

## Resources

- UUV Simulator: https://github.com/uuvsimulator/uuv_simulator
- UUV Documentation: https://uuvsimulator.github.io/
- Gazebo Sim Hydrodynamics: https://gazebosim.org/api/sim
