# AmphiReach - Autonomous Underwater Robot

[Add your project description here - e.g., "AmphiReach is an autonomous underwater robot designed for [purpose]. It features dual-hull catamaran design with differential thrust control, hydrodynamics simulation, and mission automation capabilities."]

## 🚀 Quick Start

### Prerequisites
- Ubuntu 22.04 (or compatible Linux distribution)
- Gazebo (gz-sim)
- ROS2 Jazzy (optional, for ROS2 control interfaces)
- Python 3.8+

### Installation

```bash
# Clone the repository
git clone https://github.com/yourusername/AmphiReach.git
cd AmphiReach

# Install dependencies (if script exists)
bash scripts/setup/install_dependencies.sh

# Source ROS2 (if using ROS2 interfaces)
source /opt/ros/jazzy/setup.bash
```

### Running the Simulation

```bash
# Start Gazebo simulation
gz sim simulation/worlds/ar_robot_water_world_hydrodynamics.sdf

# In another terminal, run motor controller (standalone)
python3 control/motor_control/motor_controller_standalone.py

# Or run a mission
python3 control/mission_control/mission_controller_standalone.py
```

## 📁 Project Structure

```
AmphiReach/
├── docs/              # Documentation, research, diagrams
├── models/            # 3D models (Fusion 360, meshes, URDF)
├── simulation/        # Gazebo worlds and simulation scripts
├── control/           # Motor, mission, and joint control
├── tests/             # Test files
├── scripts/           # Utility scripts
├── media/             # Images, screenshots, videos
└── data/              # Experimental results
```

See [GITHUB_STRUCTURE.md](docs/GITHUB_STRUCTURE.md) for detailed structure.

## 📚 Documentation

- [User Manual](docs/user_manual.md)
- [Research Abstract](docs/research/abstract.pdf)
- [Motor Control Guide](docs/README_MOTORS.md)
- [Mission Control Guide](docs/README_MISSION.md)
- [Project Summary](docs/PROJECT_SUMMARY.md)

## 🖼️ Media

- [Robot Photos](media/images/robot/)
- [Robot Schema](docs/diagrams/robot_schema.png)
- [Screenshots](media/images/screenshots/)

## 📊 Results

- [Results Tables](data/results/results_tables.xlsx)
- [Results Summary](data/results/results_summary.md)

## 🔧 Features

- **Dual-Hull Catamaran Design**: Stable underwater platform
- **Differential Thrust Control**: Two electric motors for precise maneuvering
- **Hydrodynamics Simulation**: Realistic underwater physics with buoyancy and drag
- **Mission Automation**: Pre-programmed mission sequences with speed and distance control
- **ROS2 Integration**: Full ROS2 support for motor and mission control
- **Standalone Control**: No ROS2 required for basic operation

## 🤖 Robot Specifications

- **Total Mass**: 215 kg
- **Motors**: 2x Electric motors
  - Continuous Power: 5.1 kW each
  - Peak Power: 8.8 kW each
  - Max Torque: 56 N·m each
  - Max RPM: 1500
  - Voltage: 48 V
- **Max Thrust**: ~560 N per motor
- **Max Speed**: 25 m/s (configurable)

## 💻 Usage Examples

### Motor Control

```bash
# Standalone (no ROS2)
python3 control/motor_control/motor_controller_standalone.py
> forward 0.5
> right 0.3
> stop

# ROS2
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}'
```

### Mission Control

```bash
# Standalone
python3 control/mission_control/mission_controller_standalone.py
> start

# ROS2
python3 control/mission_control/mission_controller.py
```

## 🧪 Testing

```bash
# Test motor control
python3 tests/test_motor_control.py

# Test mission controller
python3 tests/test_mission.py

# Test joint control
python3 tests/test_joint_control.py
```

## 📖 Research

See [Research Abstract](docs/research/abstract.pdf) for detailed research information and methodology.

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 👥 Authors

[Add your name and contact information]

## 🙏 Acknowledgments

[Add any acknowledgments]

## 📞 Contact

[Add contact information]

---

**AmphiReach** - Autonomous Underwater Robot Project
