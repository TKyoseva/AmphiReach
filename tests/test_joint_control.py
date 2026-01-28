#!/usr/bin/env python3
"""
Diagnostic script to test joint control in Gazebo
Lists available topics and tests joint commands
"""

import subprocess
import sys
import os

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
print("  Joint Control Diagnostic")
print("=" * 60)
print(f"Using gz command: {GZ_CMD}")
print()

# List all topics
print("Listing all available topics...")
print("-" * 60)
try:
    result = subprocess.run(
        [GZ_CMD, "topic", "-l"],
        capture_output=True,
        timeout=5
    )
    topics = result.stdout.decode() + result.stderr.decode()
    
    # Filter for joint-related topics
    joint_topics = [t for t in topics.split('\n') if 'joint' in t.lower() and 'ar_robot' in t.lower()]
    
    if joint_topics:
        print("Found joint topics for ar_robot:")
        for topic in joint_topics[:20]:  # Show first 20
            print(f"  {topic.strip()}")
        if len(joint_topics) > 20:
            print(f"  ... and {len(joint_topics) - 20} more")
    else:
        print("No joint topics found. Showing all topics:")
        print(topics[:1000])  # First 1000 chars
except Exception as e:
    print(f"Error listing topics: {e}")

print()
print("-" * 60)
print()

# Test specific joint topics
test_joints = [
    "base_link_Revolute-5",  # Lid
    "base_link_Revolute-6",  # Poll L
    "base_link_Revolute-7",  # Poll R
]

print("Testing joint topic formats...")
print("-" * 60)

for joint in test_joints:
    print(f"\nTesting joint: {joint}")
    
    topic_formats = [
        f"/model/ar_robot/joint/{joint}/cmd",
        f"/model/ar_robot/joint/{joint}/0/cmd",
        f"/world/ar_robot_water_world/model/ar_robot/joint/{joint}/cmd",
    ]
    
    for topic in topic_formats:
        print(f"  Trying: {topic}")
        try:
            # Try to echo the topic to see if it exists
            result = subprocess.run(
                [GZ_CMD, "topic", "-t", topic, "-e"],
                capture_output=True,
                timeout=1
            )
            if result.returncode == 0:
                print(f"    ✓ Topic exists!")
            else:
                print(f"    ✗ Topic not found or error")
        except subprocess.TimeoutExpired:
            print(f"    ? Timeout (topic might exist)")
        except Exception as e:
            print(f"    ✗ Error: {e}")

print()
print("=" * 60)
print("Diagnostic complete!")
print()
print("If no topics were found, make sure:")
print("  1. Gazebo simulation is running")
print("  2. The robot model is loaded in the simulation")
print("  3. Joint controllers are enabled (if required)")
print()
