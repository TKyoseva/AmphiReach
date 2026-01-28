#!/usr/bin/env python3
"""
Motor Force Applier - Applies forces to motors using Gazebo
Works with both ROS2 and standalone modes
"""

import subprocess
import time
import os

class MotorForceApplier:
    """Applies forces to motor links in Gazebo"""
    
    def __init__(self, gz_cmd="gz"):
        self.gz_cmd = gz_cmd
        self.world_name = "ar_robot_water_world"
        self.model_name = "ar_robot"
    
    def apply_force_to_link(self, link_name, fx, fy=0.0, fz=0.0):
        """Apply force to a link using gz service"""
        # Try multiple service path formats
        service_paths = [
            f"/world/{self.world_name}/model/{self.model_name}/link/{link_name}/apply_wrench",
            f"/world/{self.world_name}/link/{link_name}/apply_wrench",
            f"/model/{self.model_name}/link/{link_name}/apply_wrench",
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
                result = subprocess.run(
                    service_cmd,
                    capture_output=True,
                    timeout=1
                )
                if result.returncode == 0:
                    return True
            except:
                continue
        
        # Alternative: Try using topic (if available)
        try:
            topic_cmd = [
                self.gz_cmd, "topic", "-t",
                f"/world/{self.world_name}/model/{self.model_name}/link/{link_name}/wrench",
                "-m", "gz.msgs.Wrench",
                "-p", f'force: {{x: {fx:.3f}, y: {fy:.3f}, z: {fz:.3f}}}'
            ]
            subprocess.Popen(
                topic_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            return True
        except:
            pass
        
        return False

if __name__ == '__main__':
    applier = MotorForceApplier()
    print("Motor Force Applier ready")
    print("Use this module with motor controllers")
