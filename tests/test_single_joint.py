#!/usr/bin/env python3
"""
Simple test to control a single joint and see what works
"""

import subprocess
import sys
import os
import time

# Find gz command
GZ_PATHS = [
    "/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz",
    "gz",
]

GZ_CMD = None
for gz_path in GZ_PATHS:
    cmd = gz_path.split()[0]
    file_exists = os.path.exists(cmd) if os.path.isabs(cmd) else False
    
    if file_exists or not os.path.isabs(cmd):
        try:
            result = subprocess.run([cmd, "--version"], 
                                   capture_output=True, check=True, timeout=2)
            GZ_CMD = cmd
            break
        except:
            if file_exists:
                GZ_CMD = cmd
                break
            if not os.path.isabs(cmd):
                try:
                    which_result = subprocess.run(["which", cmd], 
                                                capture_output=True, timeout=1)
                    if which_result.returncode == 0:
                        GZ_CMD = cmd
                        break
                except:
                    pass

if not GZ_CMD:
    print("ERROR: gz command not found")
    sys.exit(1)

print("=" * 60)
print("  Single Joint Control Test")
print("=" * 60)
print(f"Using gz command: {GZ_CMD}")
print()
print("Make sure Gazebo simulation is running with the robot loaded!")
print("This will try to rotate the lid (base_link_Revolute-5) to 0.5 radians")
print()
input("Press Enter when simulation is ready...")

joint_name = "base_link_Revolute-5"
test_angle = 0.5

print()
print("Testing different methods...")
print("-" * 60)

# Method 1: Service call
print("\n1. Trying service call...")
try:
    service_cmd = [
        GZ_CMD, "service", "-s", "/world/ar_robot_water_world/joint/cmd",
        "--reqtype", "gz.msgs.JointCmd",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "1000",
        "--req", f'name: "{joint_name}", position: {{target: {test_angle}}}'
    ]
    result = subprocess.run(service_cmd, capture_output=True, timeout=2)
    print(f"   Return code: {result.returncode}")
    if result.stdout:
        print(f"   Output: {result.stdout.decode()[:200]}")
    if result.stderr:
        print(f"   Error: {result.stderr.decode()[:200]}")
    if result.returncode == 0:
        print("   ✓ Service call succeeded!")
    else:
        print("   ✗ Service call failed")
except Exception as e:
    print(f"   ✗ Exception: {e}")

time.sleep(1)

# Method 2: Topic without /0/
print("\n2. Trying topic without /0/...")
try:
    topic_cmd = [
        GZ_CMD, "topic", "-t", f"/model/ar_robot/joint/{joint_name}/cmd",
        "-m", "gz.msgs.Double",
        "-p", f"data: {test_angle}"
    ]
    result = subprocess.run(topic_cmd, capture_output=True, timeout=2)
    print(f"   Return code: {result.returncode}")
    if result.stdout:
        print(f"   Output: {result.stdout.decode()[:200]}")
    if result.stderr:
        print(f"   Error: {result.stderr.decode()[:200]}")
    if result.returncode == 0:
        print("   ✓ Topic command succeeded!")
    else:
        print("   ✗ Topic command failed")
except Exception as e:
    print(f"   ✗ Exception: {e}")

time.sleep(1)

# Method 3: Topic with /0/
print("\n3. Trying topic with /0/...")
try:
    topic_cmd2 = [
        GZ_CMD, "topic", "-t", f"/model/ar_robot/joint/{joint_name}/0/cmd",
        "-m", "gz.msgs.Double",
        "-p", f"data: {test_angle}"
    ]
    result = subprocess.run(topic_cmd2, capture_output=True, timeout=2)
    print(f"   Return code: {result.returncode}")
    if result.stdout:
        print(f"   Output: {result.stdout.decode()[:200]}")
    if result.stderr:
        print(f"   Error: {result.stderr.decode()[:200]}")
    if result.returncode == 0:
        print("   ✓ Topic command succeeded!")
    else:
        print("   ✗ Topic command failed")
except Exception as e:
    print(f"   ✗ Exception: {e}")

print()
print("=" * 60)
print("Test complete!")
print()
print("Check the Gazebo window - did the lid rotate?")
print("If yes, note which method number worked.")
print("If no, the joints may need a controller plugin.")
print()
