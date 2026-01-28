#!/usr/bin/env python3
"""
Wave controller script for Gazebo simulation
Animates water surface with 1m wave height using gz-sim Python API
"""

import math
import time
try:
    import gz.msgs
    import gz.transport
    GZ_AVAILABLE = True
except ImportError:
    GZ_AVAILABLE = False
    print("Warning: gz-sim Python API not available. Install with:")
    print("  pip install gz-msgs gz-transport")

def create_pose_msg(x, y, z, roll=0, pitch=0, yaw=0):
    """Create a pose message for gz-sim"""
    if not GZ_AVAILABLE:
        return None
    
    pose_msg = gz.msgs.Pose()
    pose_msg.position.x = x
    pose_msg.position.y = y
    pose_msg.position.z = z
    pose_msg.orientation.roll = roll
    pose_msg.orientation.pitch = pitch
    pose_msg.orientation.yaw = yaw
    
    return pose_msg

def animate_waves(node, pub, model_name="water_plane", amplitude=0.5, period=5.0):
    """
    Animate water surface with waves
    amplitude: wave amplitude in meters (half of total wave height)
    period: wave period in seconds
    """
    if not GZ_AVAILABLE:
        print("gz-sim Python API not available. Cannot animate waves.")
        return
    
    print(f"Starting wave animation for {model_name}")
    print(f"  Amplitude: {amplitude}m (total height: {2*amplitude}m)")
    print(f"  Period: {period}s")
    print("Press Ctrl+C to stop")
    
    start_time = time.time()
    
    try:
        while True:
            t = time.time() - start_time
            # Calculate wave height using sine wave
            wave_height = amplitude * math.sin(2 * math.pi * t / period)
            
            # Create pose message
            pose_msg = create_pose_msg(0, 0, wave_height, 0, 0, 0)
            
            if pose_msg:
                # Publish pose update
                topic = f"/model/{model_name}/pose"
                pub.publish(pose_msg, topic)
            
            # Update at ~30 Hz
            time.sleep(1.0 / 30.0)
            
    except KeyboardInterrupt:
        print("\nStopping wave animation")

if __name__ == "__main__":
    if not GZ_AVAILABLE:
        print("gz-sim Python API not available.")
        print("For now, water surface will be static.")
        print("\nTo enable wave animation:")
        print("1. Install gz-sim Python bindings")
        print("2. Run this script while simulation is running:")
        print("   python3 ar_robot/wave_controller.py")
    else:
        # Initialize gz transport
        node = gz.transport.Node()
        pub = node.advertise(gz.msgs.Pose)
        
        # Start wave animation
        animate_waves(node, pub)
