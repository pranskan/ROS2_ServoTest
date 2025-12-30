#!/bin/bash

echo "Removing unnecessary files..."

# Remove old/test scripts
rm -f safe_start.py
rm -f start_arm.sh
rm -f path_planner.py
rm -f test_arm.py
rm -f calibrate_servo.py
rm -f initial_pi_setup.sh
rm -f requirements.txt
rm -f test_ik_positions.py

echo "✓ Cleanup complete!"
echo ""
echo "Final project files:"
ls -1 *.py *.md 2>/dev/null
echo ""
echo "You should have:"
echo "  - servo_control.py"
echo "  - teleop_keyboard.py"
echo "  - path_executor.py"
echo "  - kinematics.py"
echo "  - README.md"
