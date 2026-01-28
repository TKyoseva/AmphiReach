#!/usr/bin/env python3
"""
Gazebo Motor Force Plugin (Python-based)
Applies forces to motor links based on thrust commands
This runs as a standalone script that interfaces with Gazebo
"""

import subprocess
import time
import sys
import os

# Motor parameters
MAX_THRUST = 560.0  # N (from 56 N·m * 10 N/N·m)

# Find gz command
def find_gz_command():
    gz_paths = [
        "/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz",
        "gz",
    ]
    for path in gz_paths:
        if os.path.exists(path) if os.path.isabs(path) else True:
            try:
                result = subprocess.run(
                    [path, "--version"],
                    capture_output=True,
                    timeout=1
                )
                if result.returncode == 0:
                    return path
            except:
                continue
    return "gz"

def apply_motor_force(gz_cmd, link_name, thrust):
    """Apply force to motor link using gz service"""
    try:
        # Apply force in X direction (forward)
        service_cmd = [
            gz_cmd, "service", "-s",
            f"/world/ar_robot_water_world/model/ar_robot/link/{link_name}/apply_wrench",
            "--reqtype", "gz.msgs.Wrench",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "50",
            "--req", f'force: {{x: {thrust:.3f}, y: 0.0, z: 0.0}}'
        ]
        subprocess.Popen(
            service_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        return True
    except Exception as e:
        return False

if __name__ == '__main__':
    gz_cmd = find_gz_command()
    print(f"Using gz command: {gz_cmd}")
    print("Motor force plugin ready")
    print("This script applies forces based on ROS2 topics")
    print("Run motor_controller.py to control motors")
