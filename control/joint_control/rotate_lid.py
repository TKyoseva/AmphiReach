#!/usr/bin/env python3
"""
Python script to rotate lid 90 degrees over 5 seconds
This avoids line ending issues with bash scripts
"""

import math
import time
import subprocess
import sys
import os

JOINT_NAME = "base_link_Revolute-5"
TARGET_ANGLE = math.pi / 2  # 90 degrees in radians
DURATION = 5.0  # 5 seconds
STEPS = 50  # Number of steps for smooth animation

print("Rotating lid 90 degrees over 5 seconds...")
print(f"Joint: {JOINT_NAME}")
print(f"Target angle: 90 degrees ({TARGET_ANGLE:.4f} radians)")
print()

# Check if gz is available
GZ_PATHS = [
    "/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz",
    "gz",
    "gz sim",
    "ign",
    "ign gazebo"
]

gz_found = False
GZ_CMD = None

for gz_path in GZ_PATHS:
    cmd = gz_path.split()[0]
    # Check if file exists (for absolute paths)
    file_exists = os.path.exists(cmd) if os.path.isabs(cmd) else False
    
    if file_exists or not os.path.isabs(cmd):
        try:
            # Try to run the command with --version
            result = subprocess.run([cmd, "--version"], 
                                   capture_output=True, check=True, timeout=2)
            gz_found = True
            GZ_CMD = cmd
            print(f"Using gz command: {GZ_CMD}")
            break
        except (subprocess.CalledProcessError, FileNotFoundError, subprocess.TimeoutExpired, OSError) as e:
            # If version check fails but file exists, still try to use it
            if file_exists:
                gz_found = True
                GZ_CMD = cmd
                print(f"Using gz command: {GZ_CMD} (file exists at {cmd})")
                break
            # For non-absolute paths, try which command
            if not os.path.isabs(cmd):
                try:
                    which_result = subprocess.run(["which", cmd], 
                                                capture_output=True, timeout=1)
                    if which_result.returncode == 0:
                        gz_found = True
                        GZ_CMD = cmd
                        print(f"Using gz command: {GZ_CMD} (found in PATH)")
                        break
                except:
                    pass
            continue

if not gz_found:
    print("ERROR: gz command not found")
    print("Tried paths:")
    for path in GZ_PATHS:
        print(f"  - {path}")
    print("\nYou can also control the lid manually in the Gazebo GUI:")
    print("  1. Click on the robot model")
    print("  2. Go to 'Joints' tab")
    print("  3. Find 'base_link_Revolute-5' and adjust the position slider")
    sys.exit(1)

# Calculate step size and time
STEP_SIZE = TARGET_ANGLE / STEPS
STEP_TIME = DURATION / STEPS

current_angle = 0.0

print(f"\nMake sure Gazebo simulation is running!")
print(f"Then the lid will rotate from 0 to 90 degrees over {DURATION} seconds...")
print("Starting rotation in 2 seconds...\n")
time.sleep(2)

for i in range(1, STEPS + 1):
    current_angle += STEP_SIZE
    
    # Try gz topic first (non-blocking)
    topic_cmd = [
        GZ_CMD, "topic", "-t", f"/model/ar_robot/joint/{JOINT_NAME}/0/cmd",
        "-m", "gz.msgs.Double",
        "-p", f"data: {current_angle:.6f}"
    ]
    
    # Use Popen for non-blocking execution
    try:
        process = subprocess.Popen(topic_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        # Don't wait - just start the process
        print(f"Step {i}/{STEPS}: Angle = {current_angle:.4f} rad ({math.degrees(current_angle):.1f}°)", end='\r')
    except Exception as e:
        # Try alternative topic format without /0/
        try:
            alt_topic_cmd = [
                GZ_CMD, "topic", "-t", f"/model/ar_robot/joint/{JOINT_NAME}/cmd",
                "-m", "gz.msgs.Double",
                "-p", f"data: {current_angle:.6f}"
            ]
            subprocess.Popen(alt_topic_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        except:
            # If all else fails, try service call
            try:
                service_cmd = [
                    GZ_CMD, "service", "-s", "/world/ar_robot_water_world/joint/cmd",
                    "--reqtype", "gz.msgs.JointCmd",
                    "--reptype", "gz.msgs.Boolean",
                    "--timeout", "500",
                    "--req", f'name: "{JOINT_NAME}", position: {{target: {current_angle:.6f}}}'
                ]
                subprocess.Popen(service_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            except:
                pass
    
    time.sleep(STEP_TIME)

print(f"\n\nLid rotation complete! Final angle: {TARGET_ANGLE:.4f} rad (90°)")
