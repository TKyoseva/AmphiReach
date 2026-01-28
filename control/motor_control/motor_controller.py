#!/usr/bin/env python3
"""
ROS2 Motor Controller for AR Robot
Provides differential thrust control for left and right motors
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import math
import subprocess
import os

class MotorController(Node):
    """ROS2 node for controlling left and right motors with differential thrust"""
    
    def __init__(self):
        super().__init__('motor_controller')
        
        # Motor parameters
        self.continuous_power = 5100.0  # W
        self.peak_power = 8800.0  # W
        self.nominal_torque = 33.0  # N·m
        self.max_torque = 56.0  # N·m
        self.max_rpm = 1500.0
        self.voltage = 48.0  # V
        self.gear_ratio = 1.0
        self.rotor_inertia = 0.015  # kg·m²
        
        # Thrust parameters (converting torque to thrust)
        self.thrust_per_torque = 10.0  # N per N·m (approximate)
        self.max_thrust = self.max_torque * self.thrust_per_torque  # ~560 N
        
        # Publishers for motor thrust commands (using Gazebo services)
        # We'll use gz service calls to apply forces
        self.left_thrust_cmd_pub = self.create_publisher(
            Float64,
            'motor_left/thrust',
            10
        )
        
        self.right_thrust_cmd_pub = self.create_publisher(
            Float64,
            'motor_right/thrust',
            10
        )
        
        # Find gz command for applying forces
        self.gz_cmd = self.find_gz_command()
        
        # Subscriber for velocity commands (Twist message)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Subscribers for individual motor control
        self.left_thrust_sub = self.create_subscription(
            Float64,
            'motor_left/command',
            self.left_motor_callback,
            10
        )
        
        self.right_thrust_sub = self.create_subscription(
            Float64,
            'motor_right/command',
            self.right_motor_callback,
            10
        )
        
        # Current thrust values
        self.left_thrust = 0.0
        self.right_thrust = 0.0
        
        # Timer for publishing thrust commands
        self.timer = self.create_timer(0.1, self.publish_thrust)  # 10 Hz
        
        self.get_logger().info('Motor Controller initialized')
        self.get_logger().info(f'Max thrust per motor: {self.max_thrust:.1f} N')
        self.get_logger().info('Subscribed to: cmd_vel, motor_left/command, motor_right/command')
        self.get_logger().info('Publishing to: motor_left/thrust, motor_right/thrust')
    
    def find_gz_command(self):
        """Find the gz command path"""
        gz_paths = [
            "/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz",
            "gz",
        ]
        for path in gz_paths:
            if self.os.path.exists(path) if self.os.path.isabs(path) else True:
                try:
                    result = self.subprocess.run(
                        [path, "--version"],
                        capture_output=True,
                        timeout=1
                    )
                    if result.returncode == 0:
                        return path
                except:
                    continue
        return "gz"  # Fallback
    
    def cmd_vel_callback(self, msg):
        """
        Convert Twist command to differential thrust
        Linear.x -> forward/backward (both motors)
        Angular.z -> turning (differential thrust)
        """
        linear_x = msg.linear.x  # Forward velocity command (-1 to 1)
        angular_z = msg.angular.z  # Angular velocity command (-1 to 1)
        
        # Convert to thrust commands
        # Base thrust from linear velocity
        base_thrust = linear_x * self.max_thrust
        
        # Differential thrust from angular velocity
        # Positive angular_z = turn right (left motor more, right motor less)
        diff_thrust = angular_z * self.max_thrust * 0.5
        
        self.left_thrust = base_thrust + diff_thrust
        self.right_thrust = base_thrust - diff_thrust
        
        # Clamp to max thrust
        self.left_thrust = max(-self.max_thrust, min(self.max_thrust, self.left_thrust))
        self.right_thrust = max(-self.max_thrust, min(self.max_thrust, self.right_thrust))
        
        self.get_logger().debug(
            f'Cmd: linear={linear_x:.2f}, angular={angular_z:.2f} -> '
            f'L={self.left_thrust:.1f}N, R={self.right_thrust:.1f}N'
        )
    
    def left_motor_callback(self, msg):
        """Direct command for left motor (-1.0 to 1.0)"""
        self.left_thrust = msg.data * self.max_thrust
        self.left_thrust = max(-self.max_thrust, min(self.max_thrust, self.left_thrust))
        self.get_logger().info(f'Left motor command: {self.left_thrust:.1f} N')
    
    def right_motor_callback(self, msg):
        """Direct command for right motor (-1.0 to 1.0)"""
        self.right_thrust = msg.data * self.max_thrust
        self.right_thrust = max(-self.max_thrust, min(self.max_thrust, self.right_thrust))
        self.get_logger().info(f'Right motor command: {self.right_thrust:.1f} N')
    
    def publish_thrust(self):
        """Publish current thrust values to motors using Gazebo services"""
        # Apply force to left motor link
        self.apply_force('motor_left', self.left_thrust, 0.0, 0.0)
        
        # Apply force to right motor link
        self.apply_force('motor_right', self.right_thrust, 0.0, 0.0)
        
        # Also publish to topics for compatibility
        left_msg = Float64()
        left_msg.data = self.left_thrust
        self.left_thrust_cmd_pub.publish(left_msg)
        
        right_msg = Float64()
        right_msg.data = self.right_thrust
        self.right_thrust_cmd_pub.publish(right_msg)
    
    def apply_force(self, link_name, fx, fy, fz):
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
                        "--req", f'force: {{x: {fx:.3f}, y: {fy:.3f}, z: {fz:.3f}}}'
                    ]
                    self.subprocess.Popen(
                        service_cmd,
                        stdout=self.subprocess.PIPE,
                        stderr=self.subprocess.PIPE
                    )
                    return  # Success
                except:
                    continue
            
            # Fallback: Try topic
            try:
                topic_cmd = [
                    self.gz_cmd, "topic", "-t",
                    f"/world/ar_robot_water_world/model/ar_robot/link/{link_name}/wrench",
                    "-m", "gz.msgs.Wrench",
                    "-p", f'force: {{x: {fx:.3f}, y: {fy:.3f}, z: {fz:.3f}}}'
                ]
                self.subprocess.Popen(
                    topic_cmd,
                    stdout=self.subprocess.PIPE,
                    stderr=self.subprocess.PIPE
                )
            except:
                pass
        except Exception as e:
            # Silently fail
            pass

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
