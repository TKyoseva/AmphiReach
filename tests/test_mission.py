#!/usr/bin/env python3
"""
Test script for mission controller
Sends test commands to verify mission execution
"""

import time
import sys

def test_standalone_mission():
    """Test standalone mission controller"""
    print("Testing Standalone Mission Controller")
    print("="*60)
    
    try:
        from mission_controller_standalone import StandaloneMissionController
        
        controller = StandaloneMissionController(max_speed=25.0)
        
        print("\nStarting mission in 3 seconds...")
        time.sleep(3)
        
        controller.start_mission()
        
        # Run mission loop
        start_time = time.time()
        while controller.state.value != 'completed':
            controller.run_mission()
            time.sleep(0.1)
            
            # Safety timeout
            if time.time() - start_time > 300:  # 5 minutes max
                print("Mission timeout - stopping")
                controller.stop_mission()
                break
        
        print("\nMission test completed!")
        
    except ImportError as e:
        print(f"Error importing mission controller: {e}")
        print("Make sure you're in the correct directory")
    except Exception as e:
        print(f"Error during mission test: {e}")

def test_ros2_mission():
    """Test ROS2 mission controller"""
    print("Testing ROS2 Mission Controller")
    print("="*60)
    print("\nTo test ROS2 mission controller:")
    print("1. Start Gazebo: gz sim ar_robot/ar_robot_water_world_hydrodynamics.sdf")
    print("2. Start motor controller: python3 ar_robot/motor_controller.py")
    print("3. Start mission controller: python3 ar_robot/mission_controller.py")
    print("\nOr use services:")
    print("  ros2 service call /mission/start std_srvs/srv/Trigger")

if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'ros2':
        test_ros2_mission()
    else:
        test_standalone_mission()
