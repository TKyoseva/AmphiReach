#!/usr/bin/env python3
"""
Standalone Mission Controller (No ROS2)
Executes predefined mission sequences with different speeds, durations, and distances
Uses gz services directly for motor control
"""

import subprocess
import time
import sys
import os
import math
import io
from enum import Enum

class MissionState(Enum):
    IDLE = "idle"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"

class MissionStep:
    """Represents a single mission step"""
    def __init__(self, name, target_speed, duration=None, distance=None, description=""):
        self.name = name
        self.target_speed = target_speed  # m/s
        self.duration = duration  # seconds (None if distance-based)
        self.distance = distance  # meters (None if duration-based)
        self.description = description
        self.completed = False
        self.start_time = None
        self.start_position = None
        self.distance_traveled = 0.0

class StandaloneMissionController:
    """Standalone mission controller using gz services"""
    
    def __init__(self, max_speed=25.0):
        # Motor parameters
        self.max_thrust = 560.0  # N per motor
        self.max_speed = max_speed  # m/s
        
        # Mission state
        self.state = MissionState.IDLE
        self.current_step_index = 0
        self.mission_steps = []
        
        # Robot state (simplified - would need odometry from Gazebo)
        self.current_position = [0.0, 0.0, 0.0]
        self.current_velocity = [0.0, 0.0, 0.0]
        self.last_update_time = time.time()
        
        # Find gz command
        self.gz_cmd = self.find_gz_command()
        
        # Initialize mission
        self.initialize_mission()
        
        print("=" * 60)
        print("  AR Robot Mission Controller (Standalone)")
        print("=" * 60)
        print(f"Max speed: {self.max_speed} m/s")
        print(f"Mission steps: {len(self.mission_steps)}")
        print("=" * 60)
    
    def find_gz_command(self):
        """Find the gz command path"""
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
    
    def initialize_mission(self):
        """Initialize the mission with predefined steps"""
        self.mission_steps = [
            MissionStep(
                name="warmup",
                target_speed=5.0,  # 5 m/s
                duration=10.0,  # 10 seconds
                description="Warmup: Low speed cruise for 10 seconds"
            ),
            MissionStep(
                name="medium_speed",
                target_speed=10.0,  # 10 m/s
                duration=15.0,  # 15 seconds
                description="Medium speed: 10 m/s for 15 seconds"
            ),
            MissionStep(
                name="high_speed_distance",
                target_speed=20.0,  # 20 m/s
                distance=100.0,  # 100 meters
                description="High speed: 20 m/s for 100 meters"
            ),
            MissionStep(
                name="max_speed",
                target_speed=self.max_speed,  # Maximum speed
                duration=20.0,  # 20 seconds
                description=f"Maximum speed: {self.max_speed} m/s for 20 seconds"
            ),
            MissionStep(
                name="cooldown",
                target_speed=5.0,  # 5 m/s
                duration=10.0,  # 10 seconds
                description="Cooldown: Low speed cruise for 10 seconds"
            ),
        ]
    
    def get_robot_pose(self):
        """Get robot pose from Gazebo (simplified - would need proper service call)"""
        # This is a placeholder - in real implementation, query Gazebo for pose
        # For now, estimate based on time and velocity
        dt = time.time() - self.last_update_time
        self.current_position[0] += self.current_velocity[0] * dt
        self.current_position[1] += self.current_velocity[1] * dt
        self.current_position[2] += self.current_velocity[2] * dt
        self.last_update_time = time.time()
        return self.current_position
    
    def get_current_speed(self):
        """Get current speed magnitude"""
        return math.sqrt(
            self.current_velocity[0]**2 + 
            self.current_velocity[1]**2 + 
            self.current_velocity[2]**2
        )
    
    def speed_to_thrust(self, target_speed):
        """Convert target speed to motor thrust command"""
        current_speed = self.get_current_speed()
        speed_error = target_speed - current_speed
        
        # Proportional gain
        kp = 0.5
        thrust_command = kp * speed_error
        
        # Normalize to [-1, 1] range
        thrust_command = max(-1.0, min(1.0, thrust_command / self.max_thrust))
        
        return thrust_command
    
    def apply_force(self, link_name, fx):
        """Apply force to a link using gz service"""
        try:
            service_paths = [
                f"/world/ar_robot_water_world/model/ar_robot/link/{link_name}/apply_wrench",
                f"/world/ar_robot_water_world/link/{link_name}/apply_wrench",
                f"/model/ar_robot/link/{link_name}/apply_wrench",
            ]
            
            for service_path in service_paths:
                try:
                    service_cmd = [
                        self.gz_cmd, "service", "-s", service_path,
                        "--reqtype", "gz.msgs.Wrench",
                        "--reptype", "gz.msgs.Boolean",
                        "--timeout", "50",
                        "--req", f'force: {{x: {fx:.3f}, y: 0.0, z: 0.0}}'
                    ]
                    subprocess.Popen(
                        service_cmd,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE
                    )
                    return True
                except:
                    continue
        except:
            pass
        return False
    
    def set_forward_thrust(self, thrust_value):
        """Set equal thrust to both motors for forward motion"""
        thrust_force = thrust_value * self.max_thrust
        self.apply_force('motor_left', thrust_force)
        self.apply_force('motor_right', thrust_force)
    
    def start_mission(self):
        """Start mission execution"""
        if self.state == MissionState.RUNNING:
            print("Mission already running")
            return
        
        self.state = MissionState.RUNNING
        self.current_step_index = 0
        
        # Reset all steps
        for step in self.mission_steps:
            step.completed = False
            step.start_time = None
            step.start_position = None
            step.distance_traveled = 0.0
        
        print("\n" + "="*60)
        print("MISSION STARTED")
        print("="*60)
    
    def stop_mission(self):
        """Stop mission execution"""
        self.state = MissionState.IDLE
        self.set_forward_thrust(0.0)
        print("Mission stopped")
    
    def run_mission(self):
        """Execute the mission"""
        if self.state != MissionState.RUNNING:
            return
        
        if self.current_step_index >= len(self.mission_steps):
            self.complete_mission()
            return
        
        current_step = self.mission_steps[self.current_step_index]
        current_time = time.time()
        
        # Initialize step
        if current_step.start_time is None:
            current_step.start_time = current_time
            current_step.start_position = self.get_robot_pose().copy()
            print("\n" + "="*60)
            print(f"Starting step {self.current_step_index + 1}/{len(self.mission_steps)}: {current_step.name}")
            print(f"Description: {current_step.description}")
            print(f"Target speed: {current_step.target_speed} m/s")
            if current_step.duration:
                print(f"Duration: {current_step.duration} s")
            if current_step.distance:
                print(f"Distance: {current_step.distance} m")
            print("="*60)
        
        # Calculate distance traveled
        if current_step.start_position:
            current_pos = self.get_robot_pose()
            dx = current_pos[0] - current_step.start_position[0]
            dy = current_pos[1] - current_step.start_position[1]
            dz = current_pos[2] - current_step.start_position[2]
            current_step.distance_traveled = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        # Check step completion
        elapsed_time = current_time - current_step.start_time
        step_complete = False
        
        if current_step.duration:
            if elapsed_time >= current_step.duration:
                step_complete = True
        elif current_step.distance:
            if current_step.distance_traveled >= current_step.distance:
                step_complete = True
        
        # Control speed
        current_speed = self.get_current_speed()
        thrust_command = self.speed_to_thrust(current_step.target_speed)
        self.set_forward_thrust(thrust_command)
        
        # Log progress every second
        if int(elapsed_time * 10) % 10 == 0:
            print(
                f'Step {self.current_step_index + 1}: Speed={current_speed:.2f} m/s, '
                f'Target={current_step.target_speed:.2f} m/s, '
                f'Distance={current_step.distance_traveled:.2f} m, '
                f'Time={elapsed_time:.1f} s'
            )
        
        # Complete step
        if step_complete:
            print("\n" + "="*60)
            print(f"Step {self.current_step_index + 1} completed: {current_step.name}")
            print(f"Final speed: {current_speed:.2f} m/s")
            print(f"Distance traveled: {current_step.distance_traveled:.2f} m")
            print(f"Time elapsed: {elapsed_time:.2f} s")
            print("="*60)
            
            current_step.completed = True
            self.current_step_index += 1
            
            # Brief pause between steps
            time.sleep(1.0)
    
    def complete_mission(self):
        """Complete the mission"""
        self.state = MissionState.COMPLETED
        self.set_forward_thrust(0.0)
        
        total_distance = sum(step.distance_traveled for step in self.mission_steps)
        total_time = sum(
            (step.start_time and (time.time() - step.start_time)) or 0.0 
            for step in self.mission_steps
        )
        
        print("\n" + "="*60)
        print("MISSION COMPLETED")
        print("="*60)
        print(f"Total distance: {total_distance:.2f} m")
        print(f"Total time: {total_time:.2f} s")
        print(f"Average speed: {total_distance/total_time if total_time > 0 else 0:.2f} m/s")
        print("="*60)
        
        for i, step in enumerate(self.mission_steps):
            step_time = (step.start_time and (time.time() - step.start_time)) or 0.0
            print(f"Step {i+1} ({step.name}): {step.distance_traveled:.2f} m in {step_time:.2f} s")

def main():
    controller = StandaloneMissionController(max_speed=25.0)
    
    print("\nMission Controller Ready!")
    print("Type 'start' to begin mission, 'stop' to stop, 'quit' to exit\n")
    
    import threading
    
    def mission_loop():
        """Background loop for mission execution"""
        while True:
            if controller.state == MissionState.RUNNING:
                controller.run_mission()
            time.sleep(0.1)  # 10 Hz
    
    thread = threading.Thread(target=mission_loop, daemon=True)
    thread.start()
    
    time.sleep(0.2)
    
    # Fix encoding issues for stdin
    try:
        # Try to set stdin encoding
        if sys.stdin.encoding is None or sys.stdin.encoding.lower() != 'utf-8':
            sys.stdin = io.TextIOWrapper(sys.stdin.buffer, encoding='utf-8', errors='replace')
    except:
        pass  # If we can't fix it, continue anyway
    
    while True:
        try:
            # Use sys.stdin.readline instead of input() for better encoding handling
            sys.stdout.write("> ")
            sys.stdout.flush()
            try:
                cmd = sys.stdin.readline()
            except UnicodeDecodeError:
                # Fallback: read bytes and decode with error handling
                try:
                    cmd_bytes = sys.stdin.buffer.readline()
                    cmd = cmd_bytes.decode('utf-8', errors='replace')
                except:
                    print("\nError reading input. Please try again.")
                    continue
            
            if not cmd:
                continue
                
            cmd = cmd.strip().lower()
            
            if not cmd:
                continue
            
            if cmd == 'start':
                controller.start_mission()
            elif cmd == 'stop':
                controller.stop_mission()
            elif cmd == 'status':
                if controller.state == MissionState.RUNNING:
                    step = controller.mission_steps[controller.current_step_index]
                    print(f"Step {controller.current_step_index + 1}/{len(controller.mission_steps)}: {step.name}")
                else:
                    print(f"State: {controller.state.value}")
            elif cmd == 'quit' or cmd == 'exit':
                controller.stop_mission()
                break
            else:
                print("Commands: start, stop, status, quit")
        except KeyboardInterrupt:
            print("\n")
            controller.stop_mission()
            break
        except EOFError:
            print("\n")
            controller.stop_mission()
            break
        except UnicodeDecodeError as e:
            print(f"\nEncoding error: {e}. Please try again.")
            continue
        except Exception as e:
            print(f"\nError: {e}. Please try again.")
            continue

if __name__ == '__main__':
    main()
