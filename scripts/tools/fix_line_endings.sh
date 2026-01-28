#!/bin/bash
# Fix line endings for shell scripts
sed -i 's/\r$//' ar_robot/rotate_lid.sh
sed -i 's/\r$//' ar_robot/animate_waves.sh
sed -i 's/\r$//' ar_robot/control_polls.sh
sed -i 's/\r$//' ar_robot/run_simulation.sh
sed -i 's/\r$//' ar_robot/view_urdf_wsl.sh
echo "Fixed line endings for all shell scripts"
