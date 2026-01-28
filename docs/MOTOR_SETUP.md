# Motor Setup Complete

## ✅ What Has Been Added

### 1. Motor Links in URDF
- **motor_left**: Left motor on stern of left hull (port side)
- **motor_right**: Right motor on stern of right hull (starboard side)
- Both motors are 5 kg each with proper inertia

### 2. Motor Specifications (Per Motor)
- Continuous Power: 5.1 kW
- Peak Power: 8.8 kW
- Nominal Torque: 33 N·m
- Max Torque: 56 N·m
- Max RPM: 1500
- Voltage: 48 V
- Gear Ratio: 1.0
- Rotor Inertia: 0.015 kg·m²
- **Max Thrust: ~560 N** (calculated from max torque)

### 3. Control Interfaces

#### Option A: ROS2 Interface (Recommended)
- **File**: `motor_controller.py`
- **Topics**:
  - Subscribe: `/cmd_vel` (Twist), `/motor_left/command`, `/motor_right/command`
  - Publish: `/motor_left/thrust`, `/motor_right/thrust`
- **Features**: Differential thrust control, velocity commands

#### Option B: Standalone Interface (No ROS2)
- **File**: `motor_controller_standalone.py`
- **Features**: Interactive command-line control
- **Commands**: forward, backward, left, right, stop

### 4. Differential Thrust Control
- **Forward/Backward**: Both motors receive same thrust
- **Turn Right**: Left motor more, right motor less
- **Turn Left**: Right motor more, left motor less

## 🚀 Quick Start

### Step 1: Start Simulation
```bash
gz sim ar_robot/ar_robot_water_world_hydrodynamics.sdf
```

### Step 2: Control Motors

**Option A: ROS2 (if available)**
```bash
# Terminal 2
source /opt/ros/jazzy/setup.bash
python3 ar_robot/motor_controller.py

# Terminal 3 - Send commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}'
```

**Option B: Standalone (no ROS2)**
```bash
# Terminal 2
python3 ar_robot/motor_controller_standalone.py

# Then type commands:
> forward 0.5
> right 0.3
> stop
```

## 📋 Files Created

1. **ar_robot.urdf** - Updated with motor links and joints
2. **motor_controller.py** - ROS2 motor controller node
3. **motor_controller_standalone.py** - Standalone motor controller
4. **test_motor_control.py** - Test script for motors
5. **README_MOTORS.md** - Detailed motor documentation
6. **run_simulation_complete.sh** - Complete simulation launcher
7. **package.xml** - ROS2 package definition
8. **setup.py** - Python package setup
9. **CMakeLists.txt** - Build configuration

## 🔧 Motor Force Application

Motors apply forces using Gazebo's `apply_wrench` service:
- Force applied in X-direction (forward/backward)
- Max force: 560 N per motor
- Update rate: 10 Hz

## ⚠️ Notes

- Motors are mounted on the stern (rear) of each hull
- Forces are applied directly to motor links
- If `apply_wrench` service is not available, forces may not be applied
- Check Gazebo services: `gz service -l | grep wrench`

## 🧪 Testing

Run the test script:
```bash
python3 ar_robot/test_motor_control.py
```

This will send a sequence of commands to test the motors.
