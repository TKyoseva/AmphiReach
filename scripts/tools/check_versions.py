#!/usr/bin/env python3
"""
Python script to check system versions: Ubuntu, Gazebo, ROS, and plugins
Works in both Linux and WSL
"""

import subprocess
import sys
import os
import platform

def run_command(cmd, shell=False):
    """Run a command and return output"""
    try:
        if isinstance(cmd, str) and not shell:
            cmd = cmd.split()
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=5,
            shell=shell
        )
        return result.stdout.strip(), result.returncode == 0
    except Exception as e:
        return f"Error: {e}", False

def check_ubuntu_version():
    """Check Ubuntu/Linux version"""
    print("=" * 60)
    print("  Ubuntu/Linux Version")
    print("=" * 60)
    
    # Check /etc/os-release
    if os.path.exists('/etc/os-release'):
        with open('/etc/os-release', 'r') as f:
            for line in f:
                if '=' in line:
                    key, value = line.strip().split('=', 1)
                    value = value.strip('"')
                    if key in ['NAME', 'VERSION', 'VERSION_CODENAME', 'ID']:
                        print(f"{key}: {value}")
    
    # Kernel version
    kernel, _ = run_command('uname -r')
    print(f"Kernel: {kernel}")
    print()

def check_gazebo_version():
    """Check Gazebo version"""
    print("=" * 60)
    print("  Gazebo (gz) Version")
    print("=" * 60)
    
    # Try standard gz command
    gz_paths = [
        "gz",
        "/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz",
    ]
    
    gz_found = False
    for gz_path in gz_paths:
        if os.path.exists(gz_path) if os.path.isabs(gz_path) else True:
            version, success = run_command([gz_path, "--version"])
            if success and version:
                print(f"gz command: {gz_path}")
                print(f"Version: {version}")
                gz_found = True
                break
    
    if not gz_found:
        print("gz command not found")
    
    # Check gz sim
    print("\n--- gz sim ---")
    for gz_path in gz_paths:
        if os.path.exists(gz_path) if os.path.isabs(gz_path) else True:
            version, success = run_command([gz_path, "sim", "--version"])
            if success and version:
                print(f"gz sim version: {version}")
                break
    
    print()

def check_ros_version():
    """Check ROS version"""
    print("=" * 60)
    print("  ROS Version")
    print("=" * 60)
    
    ros_distro = os.environ.get('ROS_DISTRO', '')
    ros_version = os.environ.get('ROS_VERSION', '')
    
    if ros_distro:
        print(f"ROS_DISTRO: {ros_distro}")
        print(f"ROS_VERSION: {ros_version}")
        print(f"ROS Installation: /opt/ros/{ros_distro}")
    else:
        print("ROS_DISTRO not set (ROS may not be sourced)")
        
        # Check for installed ROS
        if os.path.exists('/opt/ros'):
            distros = [d for d in os.listdir('/opt/ros') if os.path.isdir(os.path.join('/opt/ros', d))]
            if distros:
                print(f"Found ROS distributions: {', '.join(distros)}")
            else:
                print("No ROS distributions found in /opt/ros")
        else:
            print("No /opt/ros directory found")
    
    # Check ros2 command
    print("\n--- ROS2 Command ---")
    ros2_version, success = run_command("ros2 --version")
    if success:
        print(f"ros2 version: {ros2_version}")
    else:
        print("ros2 command not found")
    
    print()

def check_python_version():
    """Check Python version"""
    print("=" * 60)
    print("  Python Version")
    print("=" * 60)
    print(f"Python: {sys.version}")
    print(f"Python executable: {sys.executable}")
    print()

def check_gazebo_plugins():
    """Check Gazebo plugins"""
    print("=" * 60)
    print("  Gazebo Plugins")
    print("=" * 60)
    
    # Check for plugin info
    gz_paths = [
        "gz",
        "/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz",
    ]
    
    for gz_path in gz_paths:
        if os.path.exists(gz_path) if os.path.isabs(gz_path) else True:
            plugin_info, success = run_command([gz_path, "plugin", "--info"])
            if success and plugin_info:
                print("Available plugins (first 20 lines):")
                print('\n'.join(plugin_info.split('\n')[:20]))
                break
    
    print("\n--- Required Plugins (from URDF) ---")
    print("Plugins used in ar_robot.urdf:")
    print("  - gz-sim-hydrodynamics-system")
    print("  - gz-sim-joint-position-controller-system")
    print("  - gz-sim-user-commands-system")
    print()

def check_installed_packages():
    """Check installed packages"""
    print("=" * 60)
    print("  Installed Packages")
    print("=" * 60)
    
    # Try dpkg (Debian/Ubuntu)
    if os.path.exists('/usr/bin/dpkg'):
        print("--- Gazebo packages (dpkg) ---")
        output, _ = run_command("dpkg -l | grep -i gazebo | grep '^ii'", shell=True)
        if output:
            for line in output.split('\n')[:10]:
                if line.strip():
                    parts = line.split()
                    if len(parts) >= 2:
                        print(f"  {parts[1]} {parts[2] if len(parts) > 2 else ''}")
        else:
            print("  No Gazebo packages found")
    
    # Try rpm (RedHat/Fedora)
    elif os.path.exists('/usr/bin/rpm'):
        print("--- Gazebo packages (rpm) ---")
        output, _ = run_command("rpm -qa | grep -i gazebo", shell=True)
        if output:
            for pkg in output.split('\n')[:10]:
                if pkg.strip():
                    print(f"  {pkg}")
        else:
            print("  No Gazebo packages found")
    
    print()

def check_environment():
    """Check environment variables"""
    print("=" * 60)
    print("  Environment Variables")
    print("=" * 60)
    
    env_vars = [
        'GAZEBO_MODEL_PATH',
        'GAZEBO_RESOURCE_PATH',
        'IGN_GAZEBO_RESOURCE_PATH',
        'ROS_DISTRO',
        'ROS_VERSION',
        'PYTHONPATH',
    ]
    
    for var in env_vars:
        value = os.environ.get(var, 'not set')
        print(f"{var}: {value}")
    
    print()

def main():
    """Main function"""
    print("\n" + "=" * 60)
    print("  System Version Information")
    print("=" * 60)
    print()
    
    check_ubuntu_version()
    check_gazebo_version()
    check_ros_version()
    check_python_version()
    check_gazebo_plugins()
    check_installed_packages()
    check_environment()
    
    print("=" * 60)
    print("  Version Check Complete")
    print("=" * 60)
    print()

if __name__ == '__main__':
    main()
