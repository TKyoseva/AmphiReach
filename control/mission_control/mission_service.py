#!/usr/bin/env python3
"""
Mission Service Interface
Provides ROS2 services for mission control
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger
from example_interfaces.srv import SetBool

class MissionService(Node):
    """Service interface for mission control"""
    
    def __init__(self):
        super().__init__('mission_service')
        
        # Services
        self.start_service = self.create_service(
            Trigger,
            'mission/start',
            self.start_mission_callback
        )
        
        self.stop_service = self.create_service(
            Trigger,
            'mission/stop',
            self.stop_mission_callback
        )
        
        self.status_service = self.create_service(
            Trigger,
            'mission/status',
            self.status_mission_callback
        )
        
        # Publisher to mission controller
        self.mission_cmd_pub = self.create_publisher(
            SetBool,
            'mission/command',
            10
        )
        
        self.get_logger().info('Mission Service initialized')
        self.get_logger().info('Services available:')
        self.get_logger().info('  /mission/start - Start mission')
        self.get_logger().info('  /mission/stop - Stop mission')
        self.get_logger().info('  /mission/status - Get mission status')
    
    def start_mission_callback(self, request, response):
        """Handle start mission service call"""
        msg = SetBool.Request()
        msg.data = True
        self.mission_cmd_pub.publish(msg)
        
        response.success = True
        response.message = "Mission start command sent"
        self.get_logger().info('Mission start requested via service')
        return response
    
    def stop_mission_callback(self, request, response):
        """Handle stop mission service call"""
        msg = SetBool.Request()
        msg.data = False
        self.mission_cmd_pub.publish(msg)
        
        response.success = True
        response.message = "Mission stop command sent"
        self.get_logger().info('Mission stop requested via service')
        return response
    
    def status_mission_callback(self, request, response):
        """Handle status service call"""
        # This would query the mission controller for status
        response.success = True
        response.message = "Mission status: Check mission controller logs"
        return response

def main(args=None):
    rclpy.init(args=args)
    mission_service = MissionService()
    
    try:
        rclpy.spin(mission_service)
    except KeyboardInterrupt:
        pass
    finally:
        mission_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
