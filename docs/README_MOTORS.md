# AR Robot Motor Control

This document describes the motor control system for the AR Robot.

## Motor Specifications

Each motor has the following parameters:
- **Continuous Power**: 5.1 kW
- **Peak Power**: 8.8 kW
- **Nominal Torque**: 33 N·m
- **Max Torque**: 56 N·m
- **Max RPM**: 1500
- **Voltage**: 48 V
- **Gear Ratio**: 1.0
- **Rotor Inertia**: 0.015 kg·m²

## Motor Placement

- **Left Motor**: Mounted on the stern (rear) of the left hull (port side)
- **Right Motor**: Mounted on the stern (rear) of the right hull (starboard side)

## Differential Thrust Control

The catamaran can turn by varying the power of the two motors:
- **Forward/Backward**: Both motors receive the same thrust command
- **Turn Right**: Left motor receives more thrust, right motor receives less
- **Turn Left**: Right motor receives more thrust, left motor receives less

## ROS2 Control Interface

### Topics

#### Subscribed Topics:
- `/cmd_vel` (geometry_msgs/Twist): Velocity command
  - `linear.x`: Forward/backward velocity (-1.0 to 1.0)
  - `angular.z`: Angular velocity for turning (-1.0 to 1.0)
  
- `/motor_left/command` (std_msgs/Float64): Direct left motor command (-1.0 to 1.0)
- `/motor_right/command` (std_msgs/Float64): Direct right motor command (-1.0 to 1.0)

#### Published Topics:
- `/motor_left/thrust` (std_msgs/Float64): Left motor thrust value (Newtons)
- `/motor_right/thrust` (std_msgs/Float64): Right motor thrust value (Newtons)

### Usage

#### 1. Start the Simulation

```bash
# Terminal 1: Start Gazebo
gz sim ar_robot/ar_robot_water_world_hydrodynamics.sdf
```

#### 2. Start the Motor Controller

```bash
# Terminal 2: Start ROS2 motor controller
source /opt/ros/jazzy/setup.bash  # If not already sourced
python3 ar_robot/motor_controller.py
```

#### 3. Control the Motors

**Option A: Using cmd_vel (Recommended)**

```bash
# Move forward at 50% speed
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}'

# Turn right while moving forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}, angular: {z: 0.5}}'

# Turn left while moving forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}, angular: {z: -0.5}}'

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

**Option B: Direct Motor Control**

```bash
# Left motor at 50%
ros2 topic pub /motor_left/command std_msgs/msg/Float64 '{data: 0.5}'

# Right motor at 30%
ros2 topic pub /motor_right/command std_msgs/msg/Float64 '{data: 0.3}'
```

**Option C: Using Test Script**

```bash
python3 ar_robot/test_motor_control.py
```

#### 4. Run Everything Together

```bash
# Use the convenience script
bash ar_robot/run_simulation_with_motors.sh
```

## Motor Force Calculation

The motor controller converts torque to thrust:
- Max torque: 56 N·m
- Thrust per torque: ~10 N per N·m
- Max thrust per motor: ~560 N

Thrust is applied in the forward direction (positive X-axis) of each motor link.

## Troubleshooting

1. **Motors not responding**: 
   - Check that `motor_controller.py` is running
   - Verify ROS2 topics are available: `ros2 topic list`
   - Check Gazebo is running and robot is loaded

2. **No force applied**:
   - Verify gz service is available: `gz service -l`
   - Check motor links exist in simulation
   - Ensure world name matches: `ar_robot_water_world`

3. **ROS2 not found**:
   - Source ROS2: `source /opt/ros/jazzy/setup.bash`
   - Install ROS2 if needed

## Files

- `motor_controller.py`: Main ROS2 node for motor control
- `test_motor_control.py`: Test script for motor commands
- `run_simulation_with_motors.sh`: Convenience script to run everything
- `ar_robot.urdf`: Contains motor link definitions
