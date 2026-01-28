# AR Robot Gazebo Simulation

This directory contains the AR Robot model and Gazebo world files.

## Files

- `ar_robot.urdf` - Robot description file (includes motors)
- `ar_robot_water_world.sdf` - Gazebo world with water and sun
- `ar_robot_water_world_hydrodynamics.sdf` - World with hydrodynamics plugin
- `model.config` - Model configuration for Gazebo
- `view_urdf_wsl.sh` - Script to view URDF in WSL
- `run_simulation.sh` - Script to run the simulation
- `run_simulation_with_animation.py` - **Complete script: runs simulation + animates lid and polls**
- `run_simulation_complete.sh` - Run simulation with motor support
- `motor_controller.py` - **ROS2 motor controller** (differential thrust)
- `motor_controller_standalone.py` - **Standalone motor controller** (no ROS2 needed)
- `mission_controller.py` - **ROS2 mission controller** (multi-segment missions)
- `mission_controller_standalone.py` - **Standalone mission controller** (no ROS2 needed)
- `test_motor_control.py` - Test script for motor control
- `test_mission.py` - Test script for mission controller
- `control_polls.sh` - Script with instructions for controlling polls
- `rotate_lid.py` - Script to rotate lid 90 degrees
- `README_MOTORS.md` - **Motor control documentation**
- `README_MISSION.md` - **Mission controller documentation**
- `MISSION_EXAMPLE.md` - **Mission controller quick start guide**

## Running the Simulation

### In WSL (Windows Subsystem for Linux):

1. Navigate to the project directory:
   ```bash
   cd /mnt/c/Users/kyose/Documents/AR2.6
   ```

2. View the URDF:
   ```bash
   bash ar_robot/view_urdf_wsl.sh
   ```

3. Launch Gazebo with the water world:
   ```bash
   gz sim ar_robot/ar_robot_water_world.sdf
   ```

### Quick Start: Run Simulation with Animation

For a complete automated demo that starts the simulation and animates the robot:
```bash
python3 ar_robot/run_simulation_with_animation.py
```

This script will:
- Start the Gazebo simulation
- Rotate the lid 90 degrees over 5 seconds
- Rotate both polls 180 degrees over 5 seconds
- Keep the simulation running until you press Ctrl+C

### Alternative: Using URDF directly

If you need to convert URDF to SDF for Gazebo:
```bash
gz sdf -p ar_robot/ar_robot.urdf > ar_robot/model.sdf
```

Then update the world file to use the converted model.

## World Features

- **Sun**: Directional light source with shadows
- **Water**: Transparent water plane at z=0
- **Robot**: AR Robot positioned at z=1 (above water)

## Robot Colors

- **Hulls** (hull_l, hull_r): Red
- **Lid**: Yellow
- **Polls** (poll_l, poll_r): Green

## Controlling the Polls

The polls can be rotated using the following methods:

### Method 1: Gazebo GUI (Easiest)

1. In the Gazebo window, click on the robot model
2. In the right panel, click on the "Joints" tab
3. You'll see the joints:
   - `base_link_Revolute-6` (poll_l - left poll)
   - `base_link_Revolute-7` (poll_r - right poll)
4. Use the velocity or position sliders to rotate the polls
5. Positive values rotate counter-clockwise, negative values rotate clockwise

### Method 2: Command Line (gz topic)

If you have gz command line tools available:
```bash
# Rotate poll_l at 1.0 rad/s
gz topic -t '/model/ar_robot/joint/base_link_Revolute-6/cmd' \
  -m gz.msgs.Double -p 'data: 1.0'

# Rotate poll_r at -1.0 rad/s (opposite direction)
gz topic -t '/model/ar_robot/joint/base_link_Revolute-7/cmd' \
  -m gz.msgs.Double -p 'data: -1.0'

# Stop both polls
gz topic -t '/model/ar_robot/joint/base_link_Revolute-6/cmd' \
  -m gz.msgs.Double -p 'data: 0.0'
gz topic -t '/model/ar_robot/joint/base_link_Revolute-7/cmd' \
  -m gz.msgs.Double -p 'data: 0.0'
```

### Method 3: Python Script (Advanced)

You can create a Python script using the gz-msgs library to control the joints programmatically.

## Mesh Files

The robot uses STL mesh files located in the `meshes/` directory:
- base_link.stl
- hull_l.stl
- hull_r.stl
- Lid.stl
- poll_l.stl
- poll_r.stl

## Joint Information

- **base_link_Revolute-5**: Lid joint (continuous, Z-axis)
- **base_link_Revolute-6**: poll_l joint (continuous, Z-axis)
- **base_link_Revolute-7**: poll_r joint (continuous, Z-axis)

All revolute joints rotate around the Z-axis (0.0 0.0 1.0).
