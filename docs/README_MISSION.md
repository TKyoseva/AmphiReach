# Mission Controller Documentation

## Overview

The Mission Controller executes predefined sequences of movement segments with different speeds, durations, and distances. It provides both ROS2 and standalone implementations.

## Mission Steps

The default mission includes:

1. **Warmup**: 5 m/s for 10 seconds
2. **Medium Speed**: 10 m/s for 15 seconds
3. **High Speed Distance**: 20 m/s for 100 meters
4. **Maximum Speed**: 25 m/s (configurable) for 20 seconds
5. **Cooldown**: 5 m/s for 10 seconds

## Usage

### Option 1: Standalone (No ROS2 Required)

```bash
# Terminal 1: Start simulation
gz sim ar_robot/ar_robot_water_world_hydrodynamics.sdf

# Terminal 2: Run mission controller
python3 ar_robot/mission_controller_standalone.py

# In the controller, type:
> start
```

### Option 2: ROS2 Interface

```bash
# Terminal 1: Start simulation
gz sim ar_robot/ar_robot_water_world_hydrodynamics.sdf

# Terminal 2: Start motor controller
source /opt/ros/jazzy/setup.bash
python3 ar_robot/motor_controller.py

# Terminal 3: Start mission controller
source /opt/ros/jazzy/setup.bash
python3 ar_robot/mission_controller.py
```

The mission will start automatically if `auto_start` parameter is set to `true`.

## Parameters

### Mission Controller Parameters

- `max_speed` (default: 25.0): Maximum speed in m/s
- `auto_start` (default: false): Automatically start mission on launch

### Mission Step Configuration

Edit `mission_controller.py` or `mission_controller_standalone.py` to modify mission steps:

```python
self.mission_steps = [
    MissionStep(
        name="step_name",
        target_speed=10.0,  # m/s
        duration=15.0,      # seconds (or None)
        distance=100.0,     # meters (or None)
        description="Step description"
    ),
    # ... more steps
]
```

## Features

### Speed Control
- Proportional control to maintain target speed
- Automatic thrust adjustment based on speed error
- Speed saturation at maximum values

### Logging
- Real-time speed, distance, and time tracking
- Step-by-step progress logging
- Mission completion summary with statistics

### Distance Tracking
- Tracks distance traveled for each step
- Supports both duration-based and distance-based steps
- Calculates total mission distance

## Output Example

```
============================================================
Starting step 1/5: warmup
Description: Warmup: Low speed cruise for 10 seconds
Target speed: 5.0 m/s
Duration: 10.0 s
============================================================
Step 1: Speed=4.85 m/s, Target=5.00 m/s, Distance=48.50 m, Time=10.0 s
============================================================
Step 1 completed: warmup
Final speed: 4.95 m/s
Distance traveled: 49.50 m
Time elapsed: 10.00 s
============================================================
```

## Customization

### Adding New Mission Steps

```python
MissionStep(
    name="custom_step",
    target_speed=15.0,
    duration=30.0,  # OR distance=200.0
    description="Custom mission segment"
)
```

### Adjusting Speed Control

Modify the `speed_to_thrust()` method to adjust:
- Proportional gain (`kp`)
- Control algorithm (PID, etc.)
- Thrust limits

### Changing Maximum Speed

```python
# In standalone version
controller = StandaloneMissionController(max_speed=30.0)

# In ROS2 version, use parameter
ros2 run ar_robot_control mission_controller --ros-args -p max_speed:=30.0
```

## Troubleshooting

1. **Mission not starting**: Check that simulation is running and motors are connected
2. **Speed not reaching target**: Adjust `kp` gain in `speed_to_thrust()` method
3. **Distance tracking inaccurate**: Ensure odometry topic is correctly configured
4. **Motors not responding**: Verify motor controller is running and topics are connected

## Files

- `mission_controller.py` - ROS2 mission controller node
- `mission_controller_standalone.py` - Standalone mission controller
- `launch/mission_controller.launch.py` - ROS2 launch file
- `run_mission.sh` - Convenience script to run mission
