#!/usr/bin/env python3
"""
Test script for motor control
Publishes test commands to motors
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import time

class MotorTester(Node):
    def __init__(self):
        super().__init__('motor_tester')
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.left_pub = self.create_publisher(Float64, 'motor_left/command', 10)
        self.right_pub = self.create_publisher(Float64, 'motor_right/command', 10)
        
        self.get_logger().info('Motor tester ready')
        self.get_logger().info('Publishing test commands...')
    
    def test_forward(self):
        """Test forward motion"""
        msg = Twist()
        msg.linear.x = 0.5  # 50% forward
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info('Sent: Forward 50%')
    
    def test_turn_right(self):
        """Test turning right"""
        msg = Twist()
        msg.linear.x = 0.3
        msg.angular.z = 0.5  # Turn right
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info('Sent: Forward 30% + Turn Right')
    
    def test_turn_left(self):
        """Test turning left"""
        msg = Twist()
        msg.linear.x = 0.3
        msg.angular.z = -0.5  # Turn left
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info('Sent: Forward 30% + Turn Left')
    
    def test_direct_motors(self):
        """Test direct motor control"""
        left_msg = Float64()
        left_msg.data = 0.5  # 50% left motor
        self.left_pub.publish(left_msg)
        
        right_msg = Float64()
        right_msg.data = 0.3  # 30% right motor (differential)
        self.right_pub.publish(right_msg)
        
        self.get_logger().info('Sent: Left=50%, Right=30%')

def main():
    rclpy.init()
    tester = MotorTester()
    
    print("\n" + "="*60)
    print("Motor Control Test")
    print("="*60)
    print("\nRunning test sequence...")
    print("(Make sure motor_controller.py is running in another terminal)\n")
    
    time.sleep(2)
    
    # Test 1: Forward
    print("Test 1: Forward motion")
    tester.test_forward()
    time.sleep(3)
    
    # Test 2: Turn right
    print("\nTest 2: Turn right")
    tester.test_turn_right()
    time.sleep(3)
    
    # Test 3: Turn left
    print("\nTest 3: Turn left")
    tester.test_turn_left()
    time.sleep(3)
    
    # Test 4: Direct motor control
    print("\nTest 4: Direct motor control")
    tester.test_direct_motors()
    time.sleep(3)
    
    # Stop
    print("\nTest 5: Stop")
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    tester.cmd_vel_pub.publish(msg)
    
    print("\nTest sequence complete!")
    
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
