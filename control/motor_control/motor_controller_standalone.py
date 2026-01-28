#!/usr/bin/env python3
"""
Standalone Motor Controller for AR Robot
Works without ROS2 - uses gz services directly
Provides differential thrust control for left and right motors
"""

import subprocess
import time
import sys
import os
import math
import signal
import io

class MotorControllerStandalone:
    """Standalone motor controller using gz services"""
    
    def __init__(self):
        # Motor parameters
        self.continuous_power = 5100.0  # W
        self.peak_power = 8800.0  # W
        self.nominal_torque = 33.0  # N·m
        self.max_torque = 56.0  # N·m
        self.max_rpm = 1500.0
        self.voltage = 48.0  # V
        self.gear_ratio = 1.0
        self.rotor_inertia = 0.015  # kg·m²
        
        # Thrust parameters
        self.thrust_per_torque = 10.0  # N per N·m
        self.max_thrust = self.max_torque * self.thrust_per_torque  # ~560 N
        
        # Current thrust values
        self.left_thrust = 0.0
        self.right_thrust = 0.0
        
        # Find gz command
        self.gz_cmd = self.find_gz_command()
        
        print("=" * 60)
        print("  AR Robot Motor Controller (Standalone)")
        print("=" * 60)
        print(f"Max thrust per motor: {self.max_thrust:.1f} N")
        print(f"Using gz command: {self.gz_cmd}")
        print()
        print("Commands:")
        print("  'forward <value>' - Move forward (0.0 to 1.0)")
        print("  'backward <value>' - Move backward (0.0 to 1.0)")
        print("  'left <value>' - Turn left (0.0 to 1.0)")
        print("  'right <value>' - Turn right (0.0 to 1.0)")
        print("  'stop' - Stop all motors")
        print("  'quit' - Exit")
        print()
    
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
    
    def apply_force(self, link_name, fx):
        """Apply force to a link using gz service or topic"""
        try:
            # Try service first
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
            
            # Fallback: Try topic
            try:
                topic_cmd = [
                    self.gz_cmd, "topic", "-t",
                    f"/world/ar_robot_water_world/model/ar_robot/link/{link_name}/wrench",
                    "-m", "gz.msgs.Wrench",
                    "-p", f'force: {{x: {fx:.3f}, y: 0.0, z: 0.0}}'
                ]
                subprocess.Popen(
                    topic_cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                return True
            except:
                pass
        except:
            pass
        return False
    
    def set_thrust(self, left, right, verbose=True):
        """Set thrust for both motors"""
        self.left_thrust = max(-self.max_thrust, min(self.max_thrust, left * self.max_thrust))
        self.right_thrust = max(-self.max_thrust, min(self.max_thrust, right * self.max_thrust))
        self.update_motors(verbose=verbose)
        if verbose:
            print(f"✓ Thrust set: L={self.left_thrust:.1f}N, R={self.right_thrust:.1f}N")
    
    def update_motors(self, verbose=False):
        """Apply current thrust values to motors"""
        self.apply_force('motor_left', self.left_thrust)
        self.apply_force('motor_right', self.right_thrust)
        if verbose:
            print(f"Thrust: L={self.left_thrust:.1f}N, R={self.right_thrust:.1f}N")
    
    def run(self):
        """Main control loop - runs silently in background"""
        try:
            while True:
                # Apply forces continuously (silently)
                self.update_motors(verbose=False)
                time.sleep(0.1)  # 10 Hz update rate
        except KeyboardInterrupt:
            pass

def main():
    controller = MotorControllerStandalone()
    
    # Setup signal handler for clean shutdown
    def signal_handler(sig, frame):
        print("\n\nStopping motors...")
        controller.set_thrust(0, 0, verbose=False)
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Interactive mode
    import threading
    
    def control_loop():
        """Background loop that continuously applies forces"""
        while True:
            try:
                controller.update_motors(verbose=False)
                time.sleep(0.1)  # 10 Hz update rate
            except:
                break
    
    # Start control loop in background
    thread = threading.Thread(target=control_loop, daemon=True)
    thread.start()
    
    # Small delay to ensure thread starts
    time.sleep(0.2)
    
    # Command interface
    print("\n" + "="*60)
    print("Motor Controller Ready!")
    print("="*60)
    print("Enter commands (or 'help' for options):\n")
    
    # Fix encoding issues for stdin
    try:
        # Try to set stdin encoding
        if sys.stdin.encoding is None or sys.stdin.encoding.lower() != 'utf-8':
            sys.stdin = io.TextIOWrapper(sys.stdin.buffer, encoding='utf-8', errors='replace')
    except:
        pass  # If we can't fix it, continue anyway
    
    while True:
        try:
            # Use sys.stdout.write to avoid buffering issues
            sys.stdout.write("> ")
            sys.stdout.flush()
            
            # Use sys.stdin.readline instead of input() for better encoding handling
            try:
                cmd_line = sys.stdin.readline()
            except UnicodeDecodeError:
                # Fallback: read bytes and decode with error handling
                try:
                    cmd_bytes = sys.stdin.buffer.readline()
                    cmd_line = cmd_bytes.decode('utf-8', errors='replace')
                except:
                    print("\nError reading input. Please try again.")
                    continue
            
            if not cmd_line:
                continue
                
            cmd = cmd_line.strip().split()
            if not cmd:
                continue
            
            command = cmd[0].lower()
            
            if command == 'quit' or command == 'exit':
                print("\nStopping motors...")
                controller.set_thrust(0, 0, verbose=False)
                print("Motors stopped. Exiting...")
                break
            elif command == 'stop':
                controller.set_thrust(0, 0)
            elif command == 'forward':
                value = float(cmd[1]) if len(cmd) > 1 else 0.5
                controller.set_thrust(value, value)
            elif command == 'backward':
                value = float(cmd[1]) if len(cmd) > 1 else 0.5
                controller.set_thrust(-value, -value)
            elif command == 'left':
                value = float(cmd[1]) if len(cmd) > 1 else 0.5
                controller.set_thrust(-value, value)  # Left backward, right forward
            elif command == 'right':
                value = float(cmd[1]) if len(cmd) > 1 else 0.5
                controller.set_thrust(value, -value)  # Left forward, right backward
            elif command == 'status':
                print(f"Current thrust: L={controller.left_thrust:.1f}N, R={controller.right_thrust:.1f}N")
            elif command == 'help':
                print("\nAvailable commands:")
                print("  forward <value>  - Move forward (0.0 to 1.0)")
                print("  backward <value> - Move backward (0.0 to 1.0)")
                print("  left <value>     - Turn left (0.0 to 1.0)")
                print("  right <value>    - Turn right (0.0 to 1.0)")
                print("  stop             - Stop all motors")
                print("  status           - Show current thrust values")
                print("  quit/exit        - Exit controller\n")
            else:
                print(f"Unknown command: {command}. Type 'help' for options.")
        except (ValueError, IndexError) as e:
            print(f"Error: Invalid input. {e}")
        except KeyboardInterrupt:
            print("\nStopping motors...")
            controller.set_thrust(0, 0, verbose=False)
            break
        except EOFError:
            print("\nExiting...")
            controller.set_thrust(0, 0, verbose=False)
            break
        except UnicodeDecodeError as e:
            print(f"\nEncoding error: {e}. Please try again.")
            continue
        except Exception as e:
            print(f"\nError: {e}. Please try again.")
            continue
    
    print("Motor controller stopped.")

if __name__ == '__main__':
    main()
