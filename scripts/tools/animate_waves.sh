#!/bin/bash
# Simple wave animation script for Gazebo
# Animates water surface with 1m wave height using gz service calls

WAVE_AMPLITUDE=0.5
WAVE_PERIOD=5.0
MODEL_NAME="water_plane"

echo "Starting wave animation for $MODEL_NAME"
echo "Wave amplitude: ${WAVE_AMPLITUDE}m (total height: 1.0m)"
echo "Wave period: ${WAVE_PERIOD}s"
echo "Press Ctrl+C to stop"
echo ""

# Check if gz is available
if ! command -v gz &> /dev/null; then
    echo "ERROR: gz command not found"
    echo "Please install Gazebo: https://gazebosim.org/docs"
    exit 1
fi

# Calculate wave position using sine function
start_time=$(date +%s.%N)

while true; do
    current_time=$(date +%s.%N)
    elapsed=$(echo "$current_time - $start_time" | bc)
    
    # Calculate wave height: A * sin(2*pi*t/T)
    wave_height=$(echo "scale=4; $WAVE_AMPLITUDE * s(2 * 3.14159 * $elapsed / $WAVE_PERIOD)" | bc -l)
    
    # Set model pose using gz service
    gz service -s /world/ar_robot_water_world/set_pose \
        --reqtype gz.msgs.Pose \
        --reptype gz.msgs.Boolean \
        --timeout 1000 \
        --req "name: \"$MODEL_NAME\", position: {x: 0, y: 0, z: $wave_height}"
    
    # Update at ~10 Hz
    sleep 0.1
done
