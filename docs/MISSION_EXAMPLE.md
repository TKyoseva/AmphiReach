# Mission Controller Example

## Quick Start

### 1. Start Simulation
```bash
gz sim ar_robot/ar_robot_water_world_hydrodynamics.sdf
```

### 2. Run Mission Controller

**Standalone (Easiest):**
```bash
python3 ar_robot/mission_controller_standalone.py
> start
```

**ROS2:**
```bash
# Terminal 2: Motor Controller
source /opt/ros/jazzy/setup.bash
python3 ar_robot/motor_controller.py

# Terminal 3: Mission Controller
source /opt/ros/jazzy/setup.bash
python3 ar_robot/mission_controller.py
```

## Mission Sequence

The default mission executes:

1. **Warmup** (10s): 5 m/s
2. **Medium Speed** (15s): 10 m/s
3. **High Speed** (100m): 20 m/s
4. **Maximum Speed** (20s): 25 m/s
5. **Cooldown** (10s): 5 m/s

**Total Duration**: ~55 seconds + distance-based segment
**Total Distance**: Variable (depends on speeds achieved)

## Customizing Mission

Edit the `initialize_mission()` method in either controller file:

```python
def initialize_mission(self):
    self.mission_steps = [
        MissionStep(
            name="slow_cruise",
            target_speed=3.0,
            duration=20.0,
            description="Slow cruise at 3 m/s"
        ),
        MissionStep(
            name="fast_sprint",
            target_speed=22.0,
            distance=150.0,
            description="Fast sprint for 150 meters"
        ),
        # Add more steps...
    ]
```

## Monitoring Progress

The controller logs:
- Current speed vs target speed
- Distance traveled
- Time elapsed
- Step completion status

Example output:
```
Step 2: Speed=9.85 m/s, Target=10.00 m/s, Distance=147.50 m, Time=15.0 s
```

## Parameters

### Standalone Version
```python
controller = StandaloneMissionController(max_speed=30.0)
```

### ROS2 Version
```bash
ros2 run ar_robot_control mission_controller --ros-args \
    -p max_speed:=30.0 \
    -p auto_start:=true
```

## Tips

1. **Speed Control**: Adjust `kp` in `speed_to_thrust()` if speed is not reaching target
2. **Distance Tracking**: Ensure odometry is available (may need to add odometry publisher)
3. **Mission Timing**: Add pauses between steps by modifying the pause duration
4. **Safety**: Always test with lower speeds first
