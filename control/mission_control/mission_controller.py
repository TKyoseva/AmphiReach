#!/usr/bin/env python3
"""
AR Robot Mission Controller
Executes predefined mission sequences with different speeds, durations, and distances
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
import time
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

class MissionController(Node):
    """ROS2 node for executing boat missions"""
    
    def __init__(self):
        super().__init__('mission_controller')
        
        # Motor parameters
        self.max_thrust = 560.0  # N per motor
        self.max_speed = 25.0  # m/s (configurable)
        
        # Mission state
        self.state = MissionState.IDLE
        self.current_step_index = 0
        self.mission_steps = []
        
        # Robot state
        self.current_position = [0.0, 0.0, 0.0]
        self.current_velocity = [0.0, 0.0, 0.0]
        self.last_odom_time = None
        
        # Publishers
        self.left_thrust_pub = self.create_publisher(
            Float64,
            'motor_left/command',
            10
        )
        
        self.right_thrust_pub = self.create_publisher(
            Float64,
            'motor_right/command',
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        # Subscribers - try multiple possible odometry topics
        self.odom_sub = None
        odom_topics = [
            '/model/ar_robot/odometry',
            '/odom',
            '/ar_robot/odometry',
        ]
        
        # Try to find available odometry topic
        for topic in odom_topics:
            try:
                self.odom_sub = self.create_subscription(
                    Odometry,
                    topic,
                    self.odom_callback,
                    10
                )
                self.get_logger().info(f'Subscribed to odometry topic: {topic}')
                break
            except:
                continue
        
        if self.odom_sub is None:
            self.get_logger().warn('No odometry topic found. Distance tracking may be inaccurate.')
            # Fallback: Use pose service or estimate from time
        
        # Timer for mission execution
        self.mission_timer = self.create_timer(0.1, self.mission_loop)  # 10 Hz
        
        # Declare parameters
        self.declare_parameter('max_speed', 25.0)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('mission_file', '')
        
        # Load max speed from parameter
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        
        # Initialize mission
        self.initialize_mission()
        
        self.get_logger().info('='*60)
        self.get_logger().info('Mission Controller initialized')
        self.get_logger().info(f'Max speed: {self.max_speed} m/s')
        self.get_logger().info(f'Mission steps: {len(self.mission_steps)}')
        self.get_logger().info('='*60)
        
        # Auto-start if configured
        if self.get_parameter('auto_start').get_parameter_value().bool_value:
            self.start_mission()
    
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
    
    def odom_callback(self, msg):
        """Update robot position and velocity from odometry"""
        self.current_position[0] = msg.pose.pose.position.x
        self.current_position[1] = msg.pose.pose.position.y
        self.current_position[2] = msg.pose.pose.position.z
        
        self.current_velocity[0] = msg.twist.twist.linear.x
        self.current_velocity[1] = msg.twist.twist.linear.y
        self.current_velocity[2] = msg.twist.twist.linear.z
        
        self.last_odom_time = time.time()
    
    def get_current_speed(self):
        """Get current speed magnitude"""
        return math.sqrt(
            self.current_velocity[0]**2 + 
            self.current_velocity[1]**2 + 
            self.current_velocity[2]**2
        )
    
    def speed_to_thrust(self, target_speed):
        """
        Convert target speed to motor thrust command
        Simple proportional control with saturation
        """
        current_speed = self.get_current_speed()
        speed_error = target_speed - current_speed
        
        # Proportional gain (adjust based on your boat's dynamics)
        kp = 0.5
        thrust_command = kp * speed_error
        
        # Normalize to [-1, 1] range
        thrust_command = max(-1.0, min(1.0, thrust_command / self.max_thrust))
        
        return thrust_command
    
    def set_motor_thrust(self, left_thrust, right_thrust):
        """Send thrust commands to motors"""
        left_msg = Float64()
        left_msg.data = max(-1.0, min(1.0, left_thrust))
        self.left_thrust_pub.publish(left_msg)
        
        right_msg = Float64()
        right_msg.data = max(-1.0, min(1.0, right_thrust))
        self.right_thrust_pub.publish(right_msg)
    
    def set_forward_thrust(self, thrust_value):
        """Set equal thrust to both motors for forward motion"""
        self.set_motor_thrust(thrust_value, thrust_value)
    
    def start_mission(self):
        """Start mission execution"""
        if self.state == MissionState.RUNNING:
            self.get_logger().warn('Mission already running')
            return
        
        self.state = MissionState.RUNNING
        self.current_step_index = 0
        self.get_logger().info('='*60)
        self.get_logger().info('MISSION STARTED')
        self.get_logger().info('='*60)
        
        # Reset all steps
        for step in self.mission_steps:
            step.completed = False
            step.start_time = None
            step.start_position = None
            step.distance_traveled = 0.0
    
    def stop_mission(self):
        """Stop mission execution"""
        self.state = MissionState.IDLE
        self.set_forward_thrust(0.0)
        self.get_logger().info('Mission stopped')
    
    def mission_loop(self):
        """Main mission execution loop"""
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
            current_step.start_position = self.current_position.copy()
            self.get_logger().info('='*60)
            self.get_logger().info(f'Starting step {self.current_step_index + 1}/{len(self.mission_steps)}: {current_step.name}')
            self.get_logger().info(f'Description: {current_step.description}')
            self.get_logger().info(f'Target speed: {current_step.target_speed} m/s')
            if current_step.duration:
                self.get_logger().info(f'Duration: {current_step.duration} s')
            if current_step.distance:
                self.get_logger().info(f'Distance: {current_step.distance} m')
            self.get_logger().info('='*60)
        
        # Calculate distance traveled
        if current_step.start_position:
            dx = self.current_position[0] - current_step.start_position[0]
            dy = self.current_position[1] - current_step.start_position[1]
            dz = self.current_position[2] - current_step.start_position[2]
            current_step.distance_traveled = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        # Check step completion
        elapsed_time = current_time - current_step.start_time
        step_complete = False
        
        if current_step.duration:
            # Duration-based step
            if elapsed_time >= current_step.duration:
                step_complete = True
        elif current_step.distance:
            # Distance-based step
            if current_step.distance_traveled >= current_step.distance:
                step_complete = True
        
        # Control speed
        current_speed = self.get_current_speed()
        thrust_command = self.speed_to_thrust(current_step.target_speed)
        self.set_forward_thrust(thrust_command)
        
        # Log progress every second
        if int(elapsed_time * 10) % 10 == 0:  # Every 1 second
            self.get_logger().info(
                f'Step {self.current_step_index + 1}: Speed={current_speed:.2f} m/s, '
                f'Target={current_step.target_speed:.2f} m/s, '
                f'Distance={current_step.distance_traveled:.2f} m, '
                f'Time={elapsed_time:.1f} s'
            )
        
        # Complete step
        if step_complete:
            self.get_logger().info('='*60)
            self.get_logger().info(f'Step {self.current_step_index + 1} completed: {current_step.name}')
            self.get_logger().info(f'Final speed: {current_speed:.2f} m/s')
            self.get_logger().info(f'Distance traveled: {current_step.distance_traveled:.2f} m')
            self.get_logger().info(f'Time elapsed: {elapsed_time:.2f} s')
            self.get_logger().info('='*60)
            
            current_step.completed = True
            self.current_step_index += 1
            
            # Brief pause between steps
            time.sleep(1.0)
    
    def complete_mission(self):
        """Complete the mission"""
        self.state = MissionState.COMPLETED
        self.set_forward_thrust(0.0)
        
        # Calculate total statistics
        total_distance = sum(step.distance_traveled for step in self.mission_steps)
        total_time = sum(
            (step.start_time and (time.time() - step.start_time)) or 0.0 
            for step in self.mission_steps
        )
        
        self.get_logger().info('='*60)
        self.get_logger().info('MISSION COMPLETED')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Total distance: {total_distance:.2f} m')
        self.get_logger().info(f'Total time: {total_time:.2f} s')
        self.get_logger().info(f'Average speed: {total_distance/total_time if total_time > 0 else 0:.2f} m/s')
        self.get_logger().info('='*60)
        
        # Log each step summary
        for i, step in enumerate(self.mission_steps):
            step_time = (step.start_time and (time.time() - step.start_time)) or 0.0
            self.get_logger().info(
                f'Step {i+1} ({step.name}): '
                f'{step.distance_traveled:.2f} m in {step_time:.2f} s'
            )

def main(args=None):
    rclpy.init(args=args)
    mission_controller = MissionController()
    
    try:
        rclpy.spin(mission_controller)
    except KeyboardInterrupt:
        mission_controller.stop_mission()
    finally:
        mission_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
