# Recommended GitHub Repository Structure

This document outlines the recommended folder structure for organizing the AR Robot project on GitHub.

## 📁 Proposed Directory Structure

```
AR-Robot/
│
├── README.md                          # Main project README with overview, setup, usage
├── LICENSE                            # MIT License file
├── .gitignore                        # Git ignore patterns
├── CONTRIBUTING.md                    # Contribution guidelines (optional)
├── CHANGELOG.md                       # Version history (optional)
│
├── docs/                              # Documentation
│   ├── README.md                      # Documentation index
│   ├── research/
│   │   ├── abstract.pdf               # Research abstract
│   │   ├── papers/                    # Research papers (if any)
│   │   └── references/                # Reference materials
│   ├── diagrams/
│   │   ├── robot_schema.png           # Principal robot schema/diagram
│   │   ├── robot_schema.svg           # Vector version (if available)
│   │   ├── system_architecture.png    # System architecture diagram
│   │   └── motor_layout.png           # Motor placement diagram
│   ├── user_manual.md                 # User manual
│   └── api_reference.md               # API documentation (if needed)
│
├── models/                            # 3D Models and CAD files
│   ├── fusion360/                     # Fusion 360 files
│   │   ├── ar_robot.f3d               # Main Fusion 360 design file
│   │   ├── ar_robot_assembly.f3d      # Assembly file
│   │   ├── components/                # Individual component files
│   │   │   ├── base_link.f3d
│   │   │   ├── hull_left.f3d
│   │   │   ├── hull_right.f3d
│   │   │   ├── lid.f3d
│   │   │   ├── poll_left.f3d
│   │   │   ├── poll_right.f3d
│   │   │   ├── motor_left.f3d
│   │   │   └── motor_right.f3d
│   │   └── README.md                  # Fusion 360 file descriptions
│   ├── meshes/                        # Exported mesh files (STL, OBJ)
│   │   ├── base_link.stl
│   │   ├── hull_l.stl
│   │   ├── hull_r.stl
│   │   ├── Lid.stl
│   │   ├── poll_l.stl
│   │   ├── poll_r.stl
│   │   └── motors/                    # Motor meshes (if separate)
│   ├── urdf/                          # Robot description files
│   │   ├── ar_robot.urdf              # Main URDF file
│   │   └── ar_robot.xacro             # Xacro version (if using)
│   └── sdf/                           # SDF model files
│       └── ar_robot.sdf
│
├── simulation/                        # Simulation files
│   ├── worlds/                        # Gazebo world files
│   │   ├── ar_robot_water_world.sdf
│   │   ├── ar_robot_water_world_hydrodynamics.sdf
│   │   └── README.md                  # World descriptions
│   ├── scripts/                       # Simulation scripts
│   │   ├── run_simulation.sh
│   │   ├── run_simulation_with_animation.py
│   │   ├── run_simulation_complete.sh
│   │   └── run_mission.sh
│   ├── fusion_simulation/             # Fusion 360 simulation scripts
│   │   ├── fusion_simulation.py       # Main Fusion simulation script
│   │   ├── fusion_physics.py          # Physics simulation
│   │   └── README.md                  # Fusion simulation docs
│   └── config/                        # Simulation configuration
│       └── model.config
│
├── control/                           # Control system files
│   ├── motor_control/
│   │   ├── motor_controller.py        # ROS2 motor controller
│   │   ├── motor_controller_standalone.py
│   │   ├── motor_force_applier.py
│   │   └── README.md
│   ├── mission_control/
│   │   ├── mission_controller.py     # ROS2 mission controller
│   │   ├── mission_controller_standalone.py
│   │   ├── mission_service.py
│   │   ├── launch/
│   │   │   └── mission_controller.launch.py
│   │   └── README.md
│   └── joint_control/
│       ├── rotate_lid.py
│       ├── control_polls.sh
│       └── README.md
│
├── tests/                             # Test files
│   ├── unit_tests/                    # Unit tests
│   ├── integration_tests/             # Integration tests
│   ├── test_motor_control.py
│   ├── test_mission.py
│   ├── test_joint_control.py
│   └── test_single_joint.py
│
├── scripts/                           # Utility scripts
│   ├── setup/                         # Setup scripts
│   │   ├── install_dependencies.sh
│   │   └── setup_environment.sh
│   ├── tools/                         # Utility tools
│   │   ├── check_versions.py
│   │   ├── check_versions.sh
│   │   └── fix_line_endings.sh
│   └── build/                         # Build scripts
│       └── build_ros2_package.sh
│
├── media/                             # Images and media
│   ├── images/
│   │   ├── robot/                     # Robot photos
│   │   │   ├── robot_front.jpg
│   │   │   ├── robot_side.jpg
│   │   │   ├── robot_top.jpg
│   │   │   ├── robot_underwater.jpg
│   │   │   └── README.md              # Image descriptions
│   │   ├── screenshots/               # Simulation screenshots
│   │   │   ├── simulation_1.png
│   │   │   └── simulation_2.png
│   │   └── diagrams/                  # Additional diagrams
│   │       └── (duplicate from docs/diagrams or link)
│   └── videos/                        # Video demonstrations (if any)
│       └── README.md
│
├── data/                              # Data files
│   ├── results/                       # Experimental results
│   │   ├── results_tables.xlsx        # Result tables in Excel
│   │   ├── results_tables.csv          # CSV version (recommended)
│   │   ├── results_summary.md          # Summary of results
│   │   └── README.md                   # Data description
│   ├── logs/                          # Log files (gitignored)
│   └── config/                        # Configuration files
│
├── ros2_package/                      # ROS2 package (if separate)
│   ├── package.xml
│   ├── setup.py
│   ├── CMakeLists.txt
│   ├── setup.cfg
│   └── resource/
│       └── ar_robot_control
│
└── .github/                           # GitHub-specific files
    ├── workflows/                     # GitHub Actions CI/CD
    │   ├── ci.yml                     # Continuous Integration
    │   └── build.yml                  # Build workflow
    └── ISSUE_TEMPLATE/                # Issue templates (optional)
        ├── bug_report.md
        └── feature_request.md
```

## 📝 File Descriptions

### Root Level Files

- **README.md**: Main project documentation with:
  - Project overview
  - Quick start guide
  - Installation instructions
  - Usage examples
  - Links to other documentation

- **LICENSE**: MIT License file

- **.gitignore**: Patterns to exclude from git:
  ```
  # Python
  __pycache__/
  *.py[cod]
  *.so
  .Python
  venv/
  *.egg-info/

  # ROS
  build/
  install/
  log/

  # IDE
  .vscode/
  .idea/
  *.swp

  # OS
  .DS_Store
  Thumbs.db

  # Data/Logs
  data/logs/
  *.log

  # Fusion 360
  *.f3d.bak
  *.f3d.lock

  # Large files
  *.zip
  *.tar.gz
  ```

### Key Directories

#### `docs/`
- All documentation files
- Research materials
- Diagrams and schematics
- User manuals

#### `models/`
- **fusion360/**: Original CAD files
- **meshes/**: Exported STL/OBJ files for simulation
- **urdf/**: Robot description files
- **sdf/**: SDF model files

#### `simulation/`
- World files
- Simulation scripts
- Fusion 360 simulation scripts
- Configuration files

#### `control/`
- Motor control
- Mission control
- Joint control
- Organized by subsystem

#### `media/`
- Robot photos
- Screenshots
- Diagrams
- Videos

#### `data/`
- Experimental results
- Excel/CSV tables
- Logs (gitignored)
- Configuration data

## 🔄 Migration Plan

### Step 1: Create New Structure
```bash
mkdir -p docs/{research,diagrams} models/{fusion360/{components},meshes,urdf,sdf}
mkdir -p simulation/{worlds,scripts,fusion_simulation,config}
mkdir -p control/{motor_control,mission_control/{launch},joint_control}
mkdir -p tests/{unit_tests,integration_tests}
mkdir -p scripts/{setup,tools,build}
mkdir -p media/{images/{robot,screenshots,diagrams},videos}
mkdir -p data/{results,logs,config}
mkdir -p .github/workflows
```

### Step 2: Move Existing Files

**From `ar_robot/` to new structure:**

```bash
# Models
mv ar_robot.urdf models/urdf/
mv ar_robot_water_world*.sdf simulation/worlds/
mv meshes/* models/meshes/

# Fusion 360 files
# Move your .f3d files to models/fusion360/

# Simulation scripts
mv run_simulation*.sh simulation/scripts/
mv run_simulation*.py simulation/scripts/
mv run_mission.sh simulation/scripts/

# Control files
mv motor_controller*.py control/motor_control/
mv mission_controller*.py control/mission_control/
mv rotate_lid.py control/joint_control/

# Tests
mv test_*.py tests/

# Scripts
mv check_versions.* scripts/tools/
mv build_ros2_package.sh scripts/build/

# Documentation
mv README*.md docs/
mv *.md docs/  # Move other markdown files
# Copy research abstract to docs/research/
# Copy diagrams to docs/diagrams/

# Media
# Copy robot photos to media/images/robot/
# Copy schema diagram to docs/diagrams/robot_schema.png

# Data
# Copy Excel results to data/results/
```

### Step 3: Update File References

After moving files, update:
- Import paths in Python files
- Relative paths in scripts
- Documentation links
- README file paths

## 📋 Recommended README.md Structure

```markdown
# AmphiReach - Autonomous Underwater Robot

[Brief project description]

## 🚀 Quick Start

[Quick setup instructions]

## 📁 Project Structure

See [GITHUB_STRUCTURE.md](docs/GITHUB_STRUCTURE.md) for detailed structure.

## 📚 Documentation

- [User Manual](docs/user_manual.md)
- [Research Abstract](docs/research/abstract.pdf)
- [Motor Control Guide](docs/README_MOTORS.md)
- [Mission Control Guide](docs/README_MISSION.md)

## 🖼️ Media

- [Robot Photos](media/images/robot/)
- [Robot Schema](docs/diagrams/robot_schema.png)
- [Screenshots](media/images/screenshots/)

## 📊 Results

- [Results Tables](data/results/results_tables.xlsx)
- [Results Summary](data/results/results_summary.md)

## 🔧 Installation

[Installation instructions]

## 💻 Usage

[Usage examples]

## 🤝 Contributing

[Contributing guidelines]

## 📄 License

[License information]
```

## ✅ Best Practices

1. **Keep root clean**: Only essential files in root
2. **Logical grouping**: Group related files together
3. **Documentation**: README in each major directory
4. **Version control**: Don't commit large binaries (use Git LFS if needed)
5. **Naming**: Use lowercase with underscores for files, lowercase for folders
6. **Git LFS**: Consider Git LFS for:
   - Large Fusion 360 files
   - High-resolution images
   - Video files
   - Large data files

## 🔗 Git LFS Setup (Optional)

For large files:
```bash
git lfs install
git lfs track "*.f3d"
git lfs track "*.png"
git lfs track "*.jpg"
git lfs track "*.xlsx"
git lfs track "*.pdf"
```

## 📦 Alternative: Simplified Structure

If you prefer a simpler structure:

```
AmphiReach/
├── README.md
├── docs/
├── models/
│   ├── fusion360/
│   ├── meshes/
│   └── urdf/
├── simulation/
├── control/
├── tests/
├── media/
└── data/
```

This keeps it simpler while maintaining organization.
