# Quick Start: Motor Control

## 🚀 Fastest Way to Run

### Step 1: Start Simulation
```bash
gz sim ar_robot/ar_robot_water_world_hydrodynamics.sdf
```

### Step 2: Control Motors (Choose One)

**Option A: Standalone (Easiest - No ROS2)**
```bash
python3 ar_robot/motor_controller_standalone.py
```
Then type commands:
- `forward 0.5` - Move forward at 50%
- `right 0.3` - Turn right
- `left 0.3` - Turn left
- `stop` - Stop
- `quit` - Exit

**Option B: ROS2 Interface**
```bash
# Terminal 2
source /opt/ros/jazzy/setup.bash
python3 ar_robot/motor_controller.py

# Terminal 3 - Send commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}'
```

## 📊 Motor Specifications

- **Max Thrust**: 560 N per motor
- **Power**: 5.1 kW continuous, 8.8 kW peak
- **Torque**: 33 N·m nominal, 56 N·m max
- **Location**: Stern (rear) of each hull

## 🎮 Control Examples

### Forward Motion
```bash
# Standalone
> forward 0.5

# ROS2
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}'
```

### Turning
```bash
# Standalone - Turn right
> right 0.5

# ROS2 - Turn right
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}, angular: {z: 0.5}}'
```

## 📁 Key Files

- `motor_controller_standalone.py` - **Start here** (no ROS2 needed)
- `motor_controller.py` - ROS2 version
- `ar_robot.urdf` - Contains motor definitions
- `README_MOTORS.md` - Full documentation

## ⚡ Troubleshooting

**Motors not moving?**
1. Check simulation is running
2. Verify motor links exist: `gz model -m ar_robot -l`
3. Try standalone controller first (simpler)

**Force not applied?**
- Check gz services: `gz service -l | grep wrench`
- Motors may need to be in water to see movement
