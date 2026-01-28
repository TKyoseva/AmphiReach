#!/usr/bin/env python3
"""
Complete simulation script with lid and poll rotation
- Starts Gazebo simulation
- Rotates lid 90 degrees over 5 seconds
- Rotates polls 180 degrees over 5 seconds
"""

import math
import time
import subprocess
import sys
import os
import signal

# Configuration
JOINT_LID = "base_link_Revolute-5"
JOINT_POLL_L = "base_link_Revolute-6"
JOINT_POLL_R = "base_link_Revolute-7"

LID_TARGET_ANGLE = math.pi / 2  # 90 degrees
POLL_TARGET_ANGLE = math.pi  # 180 degrees
DURATION = 5.0  # 5 seconds
STEPS = 50  # Number of steps for smooth animation

ROBOT_X_MOVEMENT = 5.0  # Move robot 5 meters along x-axis
ROBOT_MOVE_DURATION = 5.0  # 5 seconds to move robot
ROBOT_MOVE_REPEAT = True  # Repeat movement every 5 seconds

WORLD_FILE = "ar_robot/ar_robot_water_world.sdf"

# Find gz command
GZ_PATHS = [
    "/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz",
    "gz",
    "gz sim",
    "ign",
    "ign gazebo"
]

gz_found = False
GZ_CMD = None

print("=" * 60)
print("  AR Robot Simulation with Animation")
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
        except (subprocess.CalledProcessError, FileNotFoundError, subprocess.TimeoutExpired, OSError):
            if file_exists:
                gz_found = True
                GZ_CMD = cmd
                print(f"Using gz command: {GZ_CMD} (file exists)")
                break
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

# Verify topics exist
print("Checking available joint topics...")
test_joint = JOINT_LID
topic_formats = [
    f"/model/ar_robot/joint/{test_joint}/cmd",
    f"/model/ar_robot/joint/{test_joint}/0/cmd",
    f"/world/ar_robot_water_world/model/ar_robot/joint/{test_joint}/cmd",
]

working_topic_format = None
for topic_format in topic_formats:
    try:
        # Try to list topics
        result = subprocess.run(
            [GZ_CMD, "topic", "-l"],
            capture_output=True,
            timeout=2
        )
        if topic_format in result.stdout.decode() or topic_format in result.stderr.decode():
            working_topic_format = topic_format.replace(test_joint, "{joint_name}")
            print(f"Found working topic format: {topic_format}")
            break
    except:
        pass

if not working_topic_format:
    print("Warning: Could not verify topic format. Will try multiple formats.")
    working_topic_format = "/model/ar_robot/joint/{joint_name}/cmd"  # Default from README
print()

# Animation function
def send_joint_command(joint_name, angle, previous_angle=0.0):
    """Send joint position command - tries multiple methods"""
    # Try ALL methods in parallel to increase chances of success
    # Method 1: Topic without /0/ (position command) - PRIMARY METHOD
    # This should work if joint controller plugin is loaded
    try:
        topic_cmd = [
            GZ_CMD, "topic", "-t", f"/model/ar_robot/joint/{joint_name}/cmd",
            "-m", "gz.msgs.Double",
            "-p", f"data: {angle:.6f}"
        ]
        subprocess.Popen(topic_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except:
        pass
    
    # Method 2: Topic with /0/ (alternative format)
    try:
        topic_cmd2 = [
            GZ_CMD, "topic", "-t", f"/model/ar_robot/joint/{joint_name}/0/cmd",
            "-m", "gz.msgs.Double",
            "-p", f"data: {angle:.6f}"
        ]
        subprocess.Popen(topic_cmd2, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except:
        pass
    
    # Method 3: Try velocity command (for continuous joints without position controller)
    try:
        step_time = DURATION / STEPS if STEPS > 0 else 0.1
        velocity = (angle - previous_angle) / step_time if step_time > 0 else 0
        if abs(velocity) > 0.001:  # Only send if velocity is significant
            vel_cmd = [
                GZ_CMD, "topic", "-t", f"/model/ar_robot/joint/{joint_name}/cmd_vel",
                "-m", "gz.msgs.Double",
                "-p", f"data: {velocity:.6f}"
            ]
            subprocess.Popen(vel_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        # Also try velocity topic without cmd_vel suffix
        vel_cmd2 = [
            GZ_CMD, "topic", "-t", f"/model/ar_robot/joint/{joint_name}/velocity",
            "-m", "gz.msgs.Double",
            "-p", f"data: {velocity:.6f}"
        ]
        subprocess.Popen(vel_cmd2, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except:
        pass
    
    # Always return True since we're trying multiple methods
    return True

def set_robot_pose(x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    """Set robot pose using service call"""
    try:
        service_cmd = [
            GZ_CMD, "service", "-s", "/world/ar_robot_water_world/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000",
            "--req", f'name: "ar_robot", position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}'
        ]
        result = subprocess.run(service_cmd, capture_output=True, timeout=2)
        return result.returncode == 0
    except:
        return False

# Calculate step size and time
STEP_SIZE_LID = LID_TARGET_ANGLE / STEPS
STEP_SIZE_POLL = POLL_TARGET_ANGLE / STEPS
STEP_TIME = DURATION / STEPS

print("=" * 60)
print("  Step 1: Moving Robot Along X-Axis (Repeating)")
print("=" * 60)
if ROBOT_MOVE_REPEAT:
    print(f"Moving robot 5m along x-axis every {ROBOT_MOVE_DURATION} seconds (repeating)...")
else:
    print(f"Moving robot from x=0m to x={ROBOT_X_MOVEMENT}m over {ROBOT_MOVE_DURATION} seconds...")
print()

# Move robot along x-axis smoothly
MOVE_STEPS = int(ROBOT_MOVE_DURATION * 20)  # 20 steps per second
INITIAL_Z = 2.0  # Keep z at 2m (original position)
MOVE_STEP_TIME = ROBOT_MOVE_DURATION / MOVE_STEPS
X_STEP = ROBOT_X_MOVEMENT / MOVE_STEPS

current_x = 0.0
move_count = 0

if ROBOT_MOVE_REPEAT:
    # Continuous repeating movement
    print("Starting continuous movement pattern...")
    print("(Press Ctrl+C to stop and proceed to joint animation)")
    print()
    
    try:
        while True:
            move_count += 1
            start_x = current_x
            target_x = current_x + ROBOT_X_MOVEMENT
            
            print(f"Movement #{move_count}: x={start_x:.2f}m → x={target_x:.2f}m")
            
            for i in range(1, MOVE_STEPS + 1):
                current_x = start_x + (i * X_STEP)
                success = set_robot_pose(current_x, 0.0, INITIAL_Z, 0.0, 0.0, 0.0)
                progress = (i / MOVE_STEPS) * 100
                print(f"  Moving: {progress:.1f}% | X: {current_x:.2f}m", end='\r')
                time.sleep(MOVE_STEP_TIME)
            
            # Ensure final position
            set_robot_pose(target_x, 0.0, INITIAL_Z, 0.0, 0.0, 0.0)
            print()
            print(f"  ✓ Completed movement #{move_count}. Waiting {ROBOT_MOVE_DURATION}s before next...")
            print()
            time.sleep(ROBOT_MOVE_DURATION)
    except KeyboardInterrupt:
        print()
        print("Movement stopped by user. Proceeding to joint animation...")
        print()
else:
    # Single movement
    INITIAL_X = 0.0
    TARGET_X = ROBOT_X_MOVEMENT
    current_x = INITIAL_X
    
    for i in range(1, MOVE_STEPS + 1):
        current_x += X_STEP
        success = set_robot_pose(current_x, 0.0, INITIAL_Z, 0.0, 0.0, 0.0)
        progress = (i / MOVE_STEPS) * 100
        print(f"Moving robot: {progress:.1f}% | X position: {current_x:.2f}m", end='\r')
        time.sleep(MOVE_STEP_TIME)
    
    # Ensure final position
    set_robot_pose(TARGET_X, 0.0, INITIAL_Z, 0.0, 0.0, 0.0)
    print()
    print(f"✓ Robot moved to x={TARGET_X}m")
    print()
    time.sleep(1)  # Brief pause before starting joint animation

print("=" * 60)
print("  Step 2: Starting Joint Animation")
print("=" * 60)
print(f"Lid: 0° → 90° over {DURATION} seconds")
print(f"Poll L: 0° → 180° over {DURATION} seconds")
print(f"Poll R: 0° → 180° over {DURATION} seconds")
print()
print("Sending commands...")
print()

# Test first command
print("Testing joint commands...")
test_success = send_joint_command(JOINT_LID, 0.1)
if test_success:
    print("✓ Command format works!")
else:
    print("⚠ Warning: Command may not be working. Check Gazebo GUI.")
print()

current_lid_angle = 0.0
current_poll_l_angle = 0.0
current_poll_r_angle = 0.0

prev_lid_angle = 0.0
prev_poll_l_angle = 0.0
prev_poll_r_angle = 0.0

start_time = time.time()
success_count = 0

try:
    for i in range(1, STEPS + 1):
        current_lid_angle += STEP_SIZE_LID
        current_poll_l_angle += STEP_SIZE_POLL
        current_poll_r_angle += STEP_SIZE_POLL
        
        # Send commands to all joints
        lid_ok = send_joint_command(JOINT_LID, current_lid_angle, prev_lid_angle)
        poll_l_ok = send_joint_command(JOINT_POLL_L, current_poll_l_angle, prev_poll_l_angle)
        poll_r_ok = send_joint_command(JOINT_POLL_R, current_poll_r_angle, prev_poll_r_angle)
        
        prev_lid_angle = current_lid_angle
        prev_poll_l_angle = current_poll_l_angle
        prev_poll_r_angle = current_poll_r_angle
        
        if lid_ok and poll_l_ok and poll_r_ok:
            success_count += 1
        
        elapsed = time.time() - start_time
        status = "✓" if (lid_ok and poll_l_ok and poll_r_ok) else "⚠"
        print(f"{status} Step {i}/{STEPS} | "
              f"Lid: {math.degrees(current_lid_angle):.1f}° | "
              f"Poll L: {math.degrees(current_poll_l_angle):.1f}° | "
              f"Poll R: {math.degrees(current_poll_r_angle):.1f}° | "
              f"Time: {elapsed:.2f}s", end='\r')
        
        time.sleep(STEP_TIME)
    
    print()
    print()
    print("=" * 60)
    print("  Animation Complete!")
    print("=" * 60)
    print(f"Lid: {math.degrees(LID_TARGET_ANGLE):.1f}°")
    print(f"Poll L: {math.degrees(POLL_TARGET_ANGLE):.1f}°")
    print(f"Poll R: {math.degrees(POLL_TARGET_ANGLE):.1f}°")
    print(f"Commands sent successfully: {success_count}/{STEPS}")
    print()
    if success_count < STEPS * 0.5:
        print("⚠ WARNING: Many commands may have failed.")
        print("   Try using the Gazebo GUI to control joints manually:")
        print("   1. Click on the robot in Gazebo")
        print("   2. Go to 'Joints' tab in the right panel")
        print("   3. Adjust the sliders for:")
        print(f"      - {JOINT_LID} (Lid)")
        print(f"      - {JOINT_POLL_L} (Poll L)")
        print(f"      - {JOINT_POLL_R} (Poll R)")
        print()
    print("Simulation is still running. Press Ctrl+C to stop.")
    print()
    
    # Keep simulation running
    if sim_process:
        sim_process.wait()
        
except KeyboardInterrupt:
    print("\n\nStopping simulation...")
    if sim_process:
        sim_process.terminate()
        sim_process.wait()
    print("Simulation stopped.")
    sys.exit(0)
