# AR Robot Project - Complete File Summary

This document provides a comprehensive overview of all files in the AR Robot project, organized by category.

## 📋 Table of Contents
1. [Robot Model Files](#robot-model-files)
2. [World/Simulation Files](#worldsimulation-files)
3. [Motor Control Files](#motor-control-files)
4. [Mission Control Files](#mission-control-files)
5. [Joint Control Files](#joint-control-files)
6. [Simulation Scripts](#simulation-scripts)
7. [Test Files](#test-files)
8. [Documentation Files](#documentation-files)
9. [Build/Configuration Files](#buildconfiguration-files)
10. [Utility Scripts](#utility-scripts)
11. [Mesh Files](#mesh-files)
12. [Other Files](#other-files)

---

## 🤖 Robot Model Files

### `ar_robot.urdf`
**Purpose**: Main robot description file in URDF (Unified Robot Description Format) format.
**Contents**:
- Defines all robot links (base_link, hull_l, hull_r, Lid, poll_l, poll_r, motor_left, motor_right)
- Defines all joints (fixed joints for hulls, continuous joints for lid and polls)
- Includes inertial properties (mass, inertia tensors) for all links
- Contains Gazebo-specific plugins:
  - Hydrodynamics system for underwater physics (buoyancy and drag)
  - Joint position controller for joint control
  - User commands system for motor control
- Motor definitions with physical properties
- Visual and collision geometries referencing STL mesh files

**Reason**: Core robot model definition that Gazebo uses to simulate the robot.

---

## 🌊 World/Simulation Files

### `ar_robot_water_world.sdf`
**Purpose**: Basic Gazebo world file with water and sun.
**Contents**:
- Sun light source with shadows
- Water plane (100m x 100m) at z=0
- Ground plane at z=-10m
- Includes the AR robot model
- Basic lighting and physics settings

**Reason**: Simple world for basic simulation testing.

### `ar_robot_water_world_hydrodynamics.sdf`
**Purpose**: Enhanced world file with hydrodynamics support.
**Contents**:
- Same as `ar_robot_water_world.sdf` but configured for hydrodynamics
- Water surface with proper material properties
- Optimized for underwater simulation
- Includes robot with hydrodynamics plugins

**Reason**: World file specifically designed for testing underwater physics and motor control.

---

## ⚙️ Motor Control Files

### `motor_controller.py`
**Purpose**: ROS2 node for controlling left and right motors with differential thrust.
**Features**:
- Subscribes to `/cmd_vel` (Twist) for velocity commands
- Subscribes to `/motor_left/command` and `/motor_right/command` for direct control
- Publishes thrust commands to motors
- Implements differential thrust for turning
- Uses Gazebo services to apply forces to motor links

**Reason**: ROS2 interface for motor control, enables integration with ROS2 ecosystem.

### `motor_controller_standalone.py`
**Purpose**: Standalone motor controller (no ROS2 required).
**Features**:
- Interactive command-line interface
- Commands: forward, backward, left, right, stop, status, quit
- Uses gz services directly to apply forces
- No ROS2 dependencies

**Reason**: Simple motor control without ROS2 setup, easier for quick testing.

### `motor_force_applier.py`
**Purpose**: Utility module for applying forces to motor links.
**Features**:
- Helper functions for force application
- Multiple service path fallbacks
- Error handling for service calls

**Reason**: Reusable force application logic for motor control.

### `motor_force_plugin.py`
**Purpose**: Standalone script for applying motor forces (legacy/utility).
**Reason**: Alternative method for force application testing.

---

## 🎯 Mission Control Files

### `mission_controller.py`
**Purpose**: ROS2 node for executing predefined mission sequences.
**Features**:
- Executes multi-segment missions with different speeds, durations, and distances
- Speed control with proportional feedback
- Distance and time tracking
- Logging of mission progress
- Supports both duration-based and distance-based mission steps
- Configurable via ROS2 parameters

**Reason**: Automated mission execution with logging and monitoring.

### `mission_controller_standalone.py`
**Purpose**: Standalone mission controller (no ROS2 required).
**Features**:
- Same functionality as ROS2 version but uses gz services directly
- Interactive command interface (start, stop, status, quit)
- Mission steps: warmup, medium speed, high speed, max speed, cooldown
- Real-time progress logging

**Reason**: Mission execution without ROS2 dependencies.

### `mission_service.py`
**Purpose**: ROS2 service interface for mission control.
**Features**:
- Provides ROS2 services: `/mission/start`, `/mission/stop`, `/mission/status`
- Allows remote control of missions via service calls
- Integrates with mission controller

**Reason**: Service-based interface for mission control from other ROS2 nodes.

### `launch/mission_controller.launch.py`
**Purpose**: ROS2 launch file for mission controller.
**Features**:
- Launches mission controller node
- Launches motor controller node
- Configurable parameters (max_speed, auto_start)
- Sets up topic remappings

**Reason**: Convenient way to start all mission-related nodes together.

---

## 🔧 Joint Control Files

### `rotate_lid.py`
**Purpose**: Script to rotate the lid joint 90 degrees.
**Features**:
- Uses gz topic commands to control lid joint
- Smooth animation over specified duration
- Can be used standalone or imported

**Reason**: Simple script for testing lid joint control.

### `rotate_lid.sh`
**Purpose**: Bash script wrapper for lid rotation.
**Reason**: Convenient shell script for lid rotation.

### `control_polls.sh`
**Purpose**: Script with instructions for controlling polls.
**Reason**: Helper script with commands for poll control.

---

## 🎬 Simulation Scripts

### `run_simulation.sh`
**Purpose**: Basic script to launch Gazebo simulation.
**Features**:
- Starts Gazebo with water world
- Checks for gz command availability
- Simple launcher

**Reason**: Quick way to start simulation.

### `run_simulation_with_animation.py`
**Purpose**: Complete simulation script with robot animations.
**Features**:
- Launches Gazebo simulation
- Animates lid (90 degrees over 5 seconds)
- Animates polls (180 degrees over 5 seconds)
- Moves robot along x-axis (5m every 5 seconds, repeating)
- Comprehensive error handling

**Reason**: Automated demo script showing robot capabilities.

### `run_simulation_with_animation_velocity.py`
**Purpose**: Alternative animation script using velocity control.
**Features**:
- Velocity-based joint control
- Different control approach for testing

**Reason**: Alternative method for joint animation testing.

### `run_simulation_with_motors.sh`
**Purpose**: Script to run simulation with motor control support.
**Features**:
- Starts Gazebo simulation
- Provides instructions for motor controller
- Sets up environment

**Reason**: Convenient launcher for motor testing.

### `run_simulation_complete.sh`
**Purpose**: Complete simulation launcher with all features.
**Reason**: One-stop script for full simulation setup.

### `run_mission.sh`
**Purpose**: Script to run mission simulation.
**Features**:
- Starts Gazebo
- Provides instructions for mission controller
- Sets up mission testing environment

**Reason**: Convenient launcher for mission testing.

---

## 🧪 Test Files

### `test_motor_control.py`
**Purpose**: Test script for motor control functionality.
**Features**:
- Tests forward, backward, turning motions
- Tests direct motor control
- Sends test commands to verify motor operation

**Reason**: Automated testing of motor control system.

### `test_mission.py`
**Purpose**: Test script for mission controller.
**Features**:
- Tests standalone mission execution
- Provides instructions for ROS2 mission testing
- Validates mission controller functionality

**Reason**: Automated testing of mission controller.

### `test_joint_control.py`
**Purpose**: Diagnostic script for joint control.
**Features**:
- Lists all Gazebo topics
- Tests joint command topic formats
- Helps identify correct topic paths

**Reason**: Debugging tool for joint control issues.

### `test_single_joint.py`
**Purpose**: Test script for single joint control.
**Features**:
- Tests different methods of controlling a single joint
- Tests service calls and topic commands
- Reports success/failure of each method

**Reason**: Detailed testing of joint control methods.

---

## 📚 Documentation Files

### `README.md`
**Purpose**: Main project documentation.
**Contents**:
- Project overview
- File descriptions
- Running instructions
- Joint control methods
- World features

**Reason**: Primary documentation for the project.

### `README_MOTORS.md`
**Purpose**: Comprehensive motor control documentation.
**Contents**:
- Motor specifications
- Motor placement
- Differential thrust control
- ROS2 control interface
- Usage examples
- Troubleshooting

**Reason**: Detailed guide for motor control system.

### `README_MISSION.md`
**Purpose**: Mission controller documentation.
**Contents**:
- Mission steps overview
- Usage instructions
- Parameters
- Features
- Customization guide
- Troubleshooting

**Reason**: Complete guide for mission controller.

### `MOTOR_SETUP.md`
**Purpose**: Motor setup summary.
**Contents**:
- What has been added
- Motor specifications
- Control interfaces
- Quick start guide
- Files created

**Reason**: Quick reference for motor setup.

### `QUICK_START_MOTORS.md`
**Purpose**: Quick start guide for motors.
**Contents**:
- Fastest way to run motors
- Control examples
- Key files
- Troubleshooting tips

**Reason**: Quick reference for getting started with motors.

### `MISSION_EXAMPLE.md`
**Purpose**: Mission controller quick start guide.
**Contents**:
- Quick start instructions
- Mission sequence
- Customization examples
- Monitoring progress
- Parameters

**Reason**: Quick reference for mission controller.

### `UUV_SIMULATOR_GUIDE.md`
**Purpose**: Guide for UUV Simulator integration.
**Contents**:
- UUV Simulator overview
- Compatibility information
- Installation instructions
- Recommendations

**Reason**: Reference for underwater simulation alternatives.

### `VERSION_CHECK.md`
**Purpose**: Guide for checking system versions.
**Contents**:
- Quick commands for version checking
- Script usage
- Manual version check methods
- Expected output format

**Reason**: Help users verify their system setup.

---

## 🔨 Build/Configuration Files

### `package.xml`
**Purpose**: ROS2 package definition file.
**Contents**:
- Package metadata (name, version, description)
- Dependencies (rclpy, std_msgs, geometry_msgs, nav_msgs)
- Build tool dependencies

**Reason**: Required for ROS2 package build system.

### `setup.py`
**Purpose**: Python package setup file for ROS2.
**Contents**:
- Package configuration
- Entry points for executables
- Installation requirements

**Reason**: Defines Python package structure for ROS2.

### `CMakeLists.txt`
**Purpose**: CMake build configuration.
**Contents**:
- Build configuration
- Python script installation
- Dependencies

**Reason**: Build system configuration for ROS2 package.

### `model.config`
**Purpose**: Gazebo model configuration file.
**Contents**:
- Model metadata
- Author information
- Model description

**Reason**: Required for Gazebo to recognize the model.

### `build_ros2_package.sh`
**Purpose**: Script to build ROS2 package.
**Features**:
- Checks ROS2 environment
- Builds package using CMake
- Sets up installation

**Reason**: Convenient build script for ROS2 package.

---

## 🛠️ Utility Scripts

### `check_versions.py`
**Purpose**: Python script to check system versions.
**Features**:
- Checks Ubuntu version
- Checks Gazebo version
- Checks ROS version
- Checks Python version
- Lists Gazebo plugins
- Shows installed packages
- Displays environment variables

**Reason**: Diagnostic tool for system verification.

### `check_versions.sh`
**Purpose**: Bash script version of version checker.
**Reason**: Alternative version checking method.

### `view_urdf_wsl.sh`
**Purpose**: Script to view URDF in WSL environment.
**Reason**: Helper for viewing robot model.

### `fix_line_endings.sh`
**Purpose**: Script to fix line endings in files.
**Reason**: Utility for cross-platform compatibility.

### `animate_waves.sh`
**Purpose**: Script for animating water waves.
**Reason**: Helper for wave animation testing.

---

## 🎨 Mesh Files

Located in `meshes/` directory:

### `base_link.stl`
**Purpose**: 3D mesh for the main robot body.
**Reason**: Visual and collision geometry for base_link.

### `hull_l.stl`
**Purpose**: 3D mesh for left hull.
**Reason**: Visual and collision geometry for left hull.

### `hull_r.stl`
**Purpose**: 3D mesh for right hull.
**Reason**: Visual and collision geometry for right hull.

### `Lid.stl`
**Purpose**: 3D mesh for the lid.
**Reason**: Visual and collision geometry for lid.

### `poll_l.stl`
**Purpose**: 3D mesh for left poll.
**Reason**: Visual and collision geometry for left poll.

### `poll_r.stl`
**Purpose**: 3D mesh for right poll.
**Reason**: Visual and collision geometry for right poll.

---

## 🌊 Wave Control Files

### `wave_controller.py`
**Purpose**: Wave controller script for Gazebo.
**Features**:
- Animates water surface
- Uses gz-sim Python API
- Configurable wave parameters

**Reason**: Dynamic water surface animation.

### `wave_animation.py`
**Purpose**: Alternative wave animation script.
**Reason**: Different approach to wave animation.

---

## 📊 Other Files

### `ar_robot/ar_robot.usd`
**Purpose**: USD (Universal Scene Description) format model file.
**Reason**: Alternative model format, may be used by some tools.

### `ar_robot/configuration/*.usd`
**Purpose**: USD configuration files for different aspects.
**Contents**:
- `ar_robot_base.usd`: Base configuration
- `ar_robot_physics.usd`: Physics configuration
- `ar_robot_robot.usd`: Robot configuration
- `ar_robot_sensor.usd`: Sensor configuration

**Reason**: Modular USD model configuration.

---

## 📁 Directory Structure

```
ar_robot/
├── meshes/              # STL mesh files
├── launch/              # ROS2 launch files
├── ar_robot/           # USD model files
│   └── configuration/  # USD configuration files
├── *.py                # Python scripts
├── *.sh                # Bash scripts
├── *.md                # Documentation files
├── *.urdf              # Robot model
├── *.sdf               # World files
└── *.xml, *.txt        # Configuration files
```

---

## 🎯 Quick Reference by Function

### To Run Simulation:
- `run_simulation.sh` - Basic simulation
- `run_simulation_with_animation.py` - With animations
- `run_simulation_complete.sh` - Complete setup

### To Control Motors:
- `motor_controller_standalone.py` - Standalone (easiest)
- `motor_controller.py` - ROS2 version

### To Run Missions:
- `mission_controller_standalone.py` - Standalone
- `mission_controller.py` - ROS2 version
- `run_mission.sh` - Launcher script

### To Test:
- `test_motor_control.py` - Test motors
- `test_mission.py` - Test missions
- `test_joint_control.py` - Test joints

### To Check Versions:
- `check_versions.py` - Python version checker
- `check_versions.sh` - Bash version checker

### Documentation:
- `README.md` - Main documentation
- `README_MOTORS.md` - Motor documentation
- `README_MISSION.md` - Mission documentation
- `QUICK_START_MOTORS.md` - Quick motor guide
- `MISSION_EXAMPLE.md` - Mission examples

---

## 📝 Notes

- **ROS2 vs Standalone**: Most controllers have both ROS2 and standalone versions. Use standalone for simplicity, ROS2 for integration.
- **File Naming**: Files ending in `_standalone.py` don't require ROS2.
- **Scripts vs Modules**: `.py` files can be scripts (executable) or modules (importable).
- **World Files**: Use `ar_robot_water_world_hydrodynamics.sdf` for motor/mission testing.
- **Mesh Files**: All STL files are in the `meshes/` directory and referenced by the URDF.

---

**Last Updated**: Generated automatically
**Total Files**: ~50+ files across multiple categories
