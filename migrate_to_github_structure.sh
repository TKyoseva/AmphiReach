#!/bin/bash
# Script to migrate current project structure to GitHub-ready structure
# Run this from the project root directory

set -e

echo "=========================================="
echo "  Migrating AmphiReach to GitHub Structure"
echo "=========================================="
echo ""

# Create new directory structure
echo "Creating directory structure..."
mkdir -p docs/{research,diagrams}
mkdir -p models/{fusion360/components,meshes,urdf,sdf}
mkdir -p simulation/{worlds,scripts,fusion_simulation,config}
mkdir -p control/{motor_control,mission_control/launch,joint_control}
mkdir -p tests/{unit_tests,integration_tests}
mkdir -p scripts/{setup,tools,build}
mkdir -p media/{images/{robot,screenshots,diagrams},videos}
mkdir -p data/{results,logs,config}
mkdir -p .github/workflows

echo "✓ Directory structure created"
echo ""

# Move files (adjust paths as needed)
echo "Moving files..."

# Models
if [ -f "ar_robot.urdf" ]; then
    mv ar_robot.urdf models/urdf/
    echo "✓ Moved ar_robot.urdf"
fi

if [ -f "ar_robot_water_world.sdf" ]; then
    mv ar_robot_water_world.sdf simulation/worlds/
    echo "✓ Moved ar_robot_water_world.sdf"
fi

if [ -f "ar_robot_water_world_hydrodynamics.sdf" ]; then
    mv ar_robot_water_world_hydrodynamics.sdf simulation/worlds/
    echo "✓ Moved ar_robot_water_world_hydrodynamics.sdf"
fi

if [ -d "meshes" ]; then
    mv meshes/* models/meshes/ 2>/dev/null || true
    echo "✓ Moved mesh files"
fi

if [ -f "model.config" ]; then
    mv model.config simulation/config/
    echo "✓ Moved model.config"
fi

# Simulation scripts
if [ -f "run_simulation.sh" ]; then
    mv run_simulation.sh simulation/scripts/
    echo "✓ Moved run_simulation.sh"
fi

if [ -f "run_simulation_with_animation.py" ]; then
    mv run_simulation_with_animation.py simulation/scripts/
    echo "✓ Moved run_simulation_with_animation.py"
fi

if [ -f "run_simulation_with_animation_velocity.py" ]; then
    mv run_simulation_with_animation_velocity.py simulation/scripts/
    echo "✓ Moved run_simulation_with_animation_velocity.py"
fi

if [ -f "run_simulation_complete.sh" ]; then
    mv run_simulation_complete.sh simulation/scripts/
    echo "✓ Moved run_simulation_complete.sh"
fi

if [ -f "run_simulation_with_motors.sh" ]; then
    mv run_simulation_with_motors.sh simulation/scripts/
    echo "✓ Moved run_simulation_with_motors.sh"
fi

if [ -f "run_mission.sh" ]; then
    mv run_mission.sh simulation/scripts/
    echo "✓ Moved run_mission.sh"
fi

# Control files
if [ -f "motor_controller.py" ]; then
    mv motor_controller.py control/motor_control/
    echo "✓ Moved motor_controller.py"
fi

if [ -f "motor_controller_standalone.py" ]; then
    mv motor_controller_standalone.py control/motor_control/
    echo "✓ Moved motor_controller_standalone.py"
fi

if [ -f "motor_force_applier.py" ]; then
    mv motor_force_applier.py control/motor_control/
    echo "✓ Moved motor_force_applier.py"
fi

if [ -f "motor_force_plugin.py" ]; then
    mv motor_force_plugin.py control/motor_control/
    echo "✓ Moved motor_force_plugin.py"
fi

if [ -f "mission_controller.py" ]; then
    mv mission_controller.py control/mission_control/
    echo "✓ Moved mission_controller.py"
fi

if [ -f "mission_controller_standalone.py" ]; then
    mv mission_controller_standalone.py control/mission_control/
    echo "✓ Moved mission_controller_standalone.py"
fi

if [ -f "mission_service.py" ]; then
    mv mission_service.py control/mission_control/
    echo "✓ Moved mission_service.py"
fi

if [ -d "launch" ]; then
    mv launch/* control/mission_control/launch/ 2>/dev/null || true
    echo "✓ Moved launch files"
fi

if [ -f "rotate_lid.py" ]; then
    mv rotate_lid.py control/joint_control/
    echo "✓ Moved rotate_lid.py"
fi

if [ -f "rotate_lid.sh" ]; then
    mv rotate_lid.sh control/joint_control/
    echo "✓ Moved rotate_lid.sh"
fi

if [ -f "control_polls.sh" ]; then
    mv control_polls.sh control/joint_control/
    echo "✓ Moved control_polls.sh"
fi

# Test files
if [ -f "test_motor_control.py" ]; then
    mv test_motor_control.py tests/
    echo "✓ Moved test_motor_control.py"
fi

if [ -f "test_mission.py" ]; then
    mv test_mission.py tests/
    echo "✓ Moved test_mission.py"
fi

if [ -f "test_joint_control.py" ]; then
    mv test_joint_control.py tests/
    echo "✓ Moved test_joint_control.py"
fi

if [ -f "test_single_joint.py" ]; then
    mv test_single_joint.py tests/
    echo "✓ Moved test_single_joint.py"
fi

# Utility scripts
if [ -f "check_versions.py" ]; then
    mv check_versions.py scripts/tools/
    echo "✓ Moved check_versions.py"
fi

if [ -f "check_versions.sh" ]; then
    mv check_versions.sh scripts/tools/
    echo "✓ Moved check_versions.sh"
fi

if [ -f "fix_line_endings.sh" ]; then
    mv fix_line_endings.sh scripts/tools/
    echo "✓ Moved fix_line_endings.sh"
fi

if [ -f "build_ros2_package.sh" ]; then
    mv build_ros2_package.sh scripts/build/
    echo "✓ Moved build_ros2_package.sh"
fi

if [ -f "view_urdf_wsl.sh" ]; then
    mv view_urdf_wsl.sh scripts/tools/
    echo "✓ Moved view_urdf_wsl.sh"
fi

if [ -f "animate_waves.sh" ]; then
    mv animate_waves.sh scripts/tools/
    echo "✓ Moved animate_waves.sh"
fi

if [ -f "wave_controller.py" ]; then
    mv wave_controller.py simulation/scripts/
    echo "✓ Moved wave_controller.py"
fi

if [ -f "wave_animation.py" ]; then
    mv wave_animation.py simulation/scripts/
    echo "✓ Moved wave_animation.py"
fi

# Documentation
if [ -f "README.md" ]; then
    cp README.md docs/ 2>/dev/null || true
    echo "✓ Copied README.md to docs/"
fi

# Move other markdown files to docs
for file in *.md; do
    if [ -f "$file" ] && [ "$file" != "README.md" ]; then
        mv "$file" docs/ 2>/dev/null || true
        echo "✓ Moved $file to docs/"
    fi
done

# ROS2 package files
if [ -f "package.xml" ]; then
    mkdir -p ros2_package
    mv package.xml ros2_package/
    echo "✓ Moved package.xml"
fi

if [ -f "setup.py" ]; then
    mv setup.py ros2_package/
    echo "✓ Moved setup.py"
fi

if [ -f "CMakeLists.txt" ]; then
    mv CMakeLists.txt ros2_package/
    echo "✓ Moved CMakeLists.txt"
fi

echo ""
echo "=========================================="
echo "  Migration Complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Copy Fusion 360 files to models/fusion360/"
echo "2. Copy robot photos to media/images/robot/"
echo "3. Copy robot schema diagram to docs/diagrams/robot_schema.png"
echo "4. Copy research abstract to docs/research/abstract.pdf"
echo "5. Copy results Excel file to data/results/results_tables.xlsx"
echo "6. Review and update file paths in scripts"
echo "7. Create main README.md in root (use README_TEMPLATE.md)"
echo "8. Initialize git repository: git init"
echo "9. Add files: git add ."
echo "10. Commit: git commit -m 'Initial commit: AmphiReach project'"
echo ""
echo "Note: Make sure to:"
echo "  - Update README.md with AmphiReach project information"
echo "  - Add LICENSE file (MIT license already created)"
echo "  - Add your Fusion 360 files to models/fusion360/"
echo "  - Add robot photos to media/images/robot/"
echo "  - Add schema diagram to docs/diagrams/robot_schema.png"
echo "  - Add research abstract to docs/research/abstract.pdf"
echo "  - Add results Excel to data/results/results_tables.xlsx"
echo ""
