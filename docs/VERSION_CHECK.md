# Version Check Guide

## Quick Commands

Run these commands in your WSL terminal to check versions:

### Ubuntu Version
```bash
cat /etc/os-release | grep -E "NAME|VERSION|VERSION_CODENAME"
uname -r
```

### Gazebo (gz) Version
```bash
gz --version
gz sim --version
```

### ROS Version
```bash
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_VERSION: $ROS_VERSION"
ros2 --version
```

### Python Version
```bash
python3 --version
```

### Gazebo Plugins
```bash
gz plugin --info
```

## Using the Version Check Scripts

### Option 1: Python Script (Recommended)
```bash
cd /mnt/c/Users/kyose/Documents/AR2.6
python3 ar_robot/check_versions.py
```

### Option 2: Bash Script
```bash
cd /mnt/c/Users/kyose/Documents/AR2.6
bash ar_robot/check_versions.sh
```

## Manual Version Check

If the scripts don't work, you can check manually:

### 1. Ubuntu Version
```bash
lsb_release -a
# or
cat /etc/os-release
```

### 2. Gazebo Version
```bash
# Standard path
gz --version

# Or if in ROS installation
/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz --version
```

### 3. ROS Version
```bash
# Check if ROS is sourced
echo $ROS_DISTRO

# List installed ROS distributions
ls /opt/ros/

# Check ROS2 version
ros2 --version
```

### 4. Installed Packages
```bash
# Gazebo packages
dpkg -l | grep gazebo

# ROS packages
dpkg -l | grep ros
```

### 5. Environment Variables
```bash
env | grep -E "GAZEBO|ROS|IGN"
```

## Expected Output Format

When you run the scripts, you should see:

```
============================================================
  System Version Information
============================================================

============================================================
  Ubuntu/Linux Version
============================================================
NAME: Ubuntu
VERSION: 22.04.x LTS (Jammy Jellyfish)
VERSION_CODENAME: jammy
ID: ubuntu
Kernel: 5.x.x-x-generic

============================================================
  Gazebo (gz) Version
============================================================
gz command: gz
Version: gz (Gazebo) X.X.X

============================================================
  ROS Version
============================================================
ROS_DISTRO: jazzy
ROS_VERSION: 2
ros2 version: ros2 X.X.X

============================================================
  Python Version
============================================================
Python: 3.x.x
Python executable: /usr/bin/python3

============================================================
  Gazebo Plugins
============================================================
[Plugin information...]

============================================================
  Installed Packages
============================================================
[Package list...]

============================================================
  Environment Variables
============================================================
GAZEBO_MODEL_PATH: ...
ROS_DISTRO: jazzy
...
```

## Troubleshooting

If scripts fail:
1. Make sure you're in WSL, not Windows PowerShell
2. Ensure Python3 is installed: `python3 --version`
3. Check file permissions: `ls -l ar_robot/check_versions.*`
4. Run manually using the commands above
