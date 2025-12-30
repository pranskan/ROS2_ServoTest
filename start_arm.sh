#!/bin/bash

echo "================================================"
echo "Starting Robotic Arm Control"
echo "================================================"

# Source ROS2 environment
echo "Sourcing ROS2 environment..."
source /opt/ros/jazzy/setup.bash

# Source virtual environment
echo "Activating Python virtual environment..."
source ~/ros2_servo_venv/bin/activate

# Clear any stale topics (optional - helps prevent jerky startup)
echo "Cleaning up any stale ROS2 topics..."
# Kill any existing servo_control nodes
pkill -f servo_control.py 2>/dev/null

# Wait a moment for cleanup
sleep 1

echo ""
echo "Starting servo_control node..."
echo "================================================"
echo ""

# Start servo control with output
python3 servo_control.py
