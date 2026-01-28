#!/bin/bash
# Script to rotate lid 90 degrees over 5 seconds

JOINT_NAME="base_link_Revolute-5"
TARGET_ANGLE=1.5708
DURATION=5.0
STEPS=50

echo "Rotating lid 90 degrees over 5 seconds..."
echo "Joint: $JOINT_NAME"
echo "Target angle: 90 degrees ($TARGET_ANGLE radians)"
echo ""

if ! command -v gz &> /dev/null; then
    echo "ERROR: gz command not found"
    exit 1
fi

STEP_SIZE=$(echo "scale=6; $TARGET_ANGLE / $STEPS" | bc)
STEP_TIME=$(echo "scale=3; $DURATION / $STEPS" | bc)

current_angle=0.0

for i in $(seq 1 $STEPS); do
    current_angle=$(echo "scale=6; $current_angle + $STEP_SIZE" | bc)
    
    gz topic -t "/model/ar_robot/joint/${JOINT_NAME}/0/cmd" \
        -m gz.msgs.Double \
        -p "data: $current_angle" 2>/dev/null || \
    gz service -s /world/ar_robot_water_world/joint/cmd \
        --reqtype gz.msgs.JointCmd \
        --reptype gz.msgs.Boolean \
        --timeout 1000 \
        --req "name: \"$JOINT_NAME\", position: {target: $current_angle}" 2>/dev/null
    
    sleep $STEP_TIME
done

echo "Lid rotation complete!"
