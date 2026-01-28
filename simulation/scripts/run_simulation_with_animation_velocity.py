#!/usr/bin/env python3
"""
Alternative simulation script using VELOCITY commands for continuous joints
- Starts Gazebo simulation
- Rotates lid 90 degrees over 5 seconds using velocity
- Rotates polls 180 degrees over 5 seconds using velocity
"""

import math
import time
import subprocess
import sys
import os

# Configuration
JOINT_LID = "base_link_Revolute-5"
JOINT_POLL_L = "base_link_Revolute-6"
JOINT_POLL_R = "base_link_Revolute-7"

LID_TARGET_ANGLE = math.pi / 2  # 90 degrees
POLL_TARGET_ANGLE = math.pi  # 180 degrees
DURATION = 5.0  # 5 seconds
STEPS = 50  # Number of steps for smooth animation

WORLD_FILE = "ar_robot/ar_robot_water_world.sdf"

# Find gz command
GZ_PATHS = [
    "/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz",
    "gz",
]

gz_found = False
GZ_CMD = None

print("=" * 60)
print("  AR Robot Simulation with Velocity Control")
print("=" * 60)
print()

# Find gz command
for gz_path in GZ_PATHS:
    cmd = gz_path.split()[0]
    file_exists = os.path.exists(cmd) if os.path.isabs(cmd) else False
    
    if file_exists or not os.path.isabs(cmd):
        try:
            result = subprocess.run([cmd, "--version"], 
                                   capture_output=True, check=True, timeout=2)
            gz_found = True
            GZ_CMD = cmd
            print(f"Using gz command: {GZ_CMD}")
            break
        except:
            if file_exists:
                gz_found = True
                GZ_CMD = cmd
                break
            if not os.path.isabs(cmd):
                try:
                    which_result = subprocess.run(["which", cmd], 
                                                capture_output=True, timeout=1)
                    if which_result.returncode == 0:
                        gz_found = True
                        GZ_CMD = cmd
                        break
                except:
                    pass

if not gz_found:
    print("ERROR: gz command not found")
    sys.exit(1)

# Check if world file exists
if not os.path.exists(WORLD_FILE):
    print(f"ERROR: World file not found: {WORLD_FILE}")
    sys.exit(1)

print(f"World file: {WORLD_FILE}")
print()

# Start simulation
print("Starting Gazebo simulation...")
print("(This will open a new window)")
print()

sim_process = None
try:
    sim_cmd = [GZ_CMD, "sim", WORLD_FILE]
    sim_process = subprocess.Popen(sim_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    print("Simulation started! Waiting 5 seconds for it to initialize...")
    time.sleep(5)
    print()
except Exception as e:
    print(f"ERROR: Failed to start simulation: {e}")
    sys.exit(1)

# Calculate target velocities
LID_VELOCITY = LID_TARGET_ANGLE / DURATION  # rad/s to reach target in DURATION seconds
POLL_VELOCITY = POLL_TARGET_ANGLE / DURATION  # rad/s

def send_velocity_command(joint_name, velocity):
    """Send joint velocity command"""
    # Try multiple topic formats
    topics = [
        f"/model/ar_robot/joint/{joint_name}/cmd_vel",
        f"/model/ar_robot/joint/{joint_name}/velocity",
        f"/model/ar_robot/joint/{joint_name}/0/cmd_vel",
        f"/model/ar_robot/joint/{joint_name}/0/velocity",
        f"/model/ar_robot/joint/{joint_name}/cmd",  # Some systems use cmd for velocity
    ]
    
    for topic in topics:
        try:
            cmd = [
                GZ_CMD, "topic", "-t", topic,
                "-m", "gz.msgs.Double",
                "-p", f"data: {velocity:.6f}"
            ]
            subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        except:
            pass

print("=" * 60)
print("  Starting Animation (Velocity Control)")
print("=" * 60)
print(f"Lid: rotating at {math.degrees(LID_VELOCITY):.2f}°/s for {DURATION}s (target: 90°)")
print(f"Poll L: rotating at {math.degrees(POLL_VELOCITY):.2f}°/s for {DURATION}s (target: 180°)")
print(f"Poll R: rotating at {math.degrees(POLL_VELOCITY):.2f}°/s for {DURATION}s (target: 180°)")
print()

# Start all joints rotating
print("Starting rotation...")
send_velocity_command(JOINT_LID, LID_VELOCITY)
send_velocity_command(JOINT_POLL_L, POLL_VELOCITY)
send_velocity_command(JOINT_POLL_R, POLL_VELOCITY)

start_time = time.time()
print("Rotating... (check Gazebo window)")

# Let them rotate for DURATION seconds
try:
    while time.time() - start_time < DURATION:
        elapsed = time.time() - start_time
        progress = min(100, (elapsed / DURATION) * 100)
        print(f"Progress: {progress:.1f}% | Time: {elapsed:.2f}s", end='\r')
        time.sleep(0.1)
    
    print()
    print()
    
    # Stop all joints
    print("Stopping joints...")
    send_velocity_command(JOINT_LID, 0.0)
    send_velocity_command(JOINT_POLL_L, 0.0)
    send_velocity_command(JOINT_POLL_R, 0.0)
    
    print()
    print("=" * 60)
    print("  Animation Complete!")
    print("=" * 60)
    print(f"Lid should be at: {math.degrees(LID_TARGET_ANGLE):.1f}°")
    print(f"Poll L should be at: {math.degrees(POLL_TARGET_ANGLE):.1f}°")
    print(f"Poll R should be at: {math.degrees(POLL_TARGET_ANGLE):.1f}°")
    print()
    print("Simulation is still running. Press Ctrl+C to stop.")
    print()
    
    # Keep simulation running
    if sim_process:
        sim_process.wait()
        
except KeyboardInterrupt:
    print("\n\nStopping joints and simulation...")
    send_velocity_command(JOINT_LID, 0.0)
    send_velocity_command(JOINT_POLL_L, 0.0)
    send_velocity_command(JOINT_POLL_R, 0.0)
    if sim_process:
        sim_process.terminate()
        sim_process.wait()
    print("Simulation stopped.")
    sys.exit(0)
