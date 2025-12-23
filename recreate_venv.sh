#!/bin/bash

echo "=== Recreating ROS2 Servo Virtual Environment ==="
echo ""
echo "⚠️  WARNING: This will DELETE and recreate your virtual environment!"
echo ""
echo "Only run this if:"
echo "  - Packages are broken/corrupted"
echo "  - Initial installation failed"
echo "  - You want to update package versions"
echo ""
read -p "Continue? (y/n): " choice
case "$choice" in 
    y|Y ) echo "Proceeding...";;
    * ) echo "Cancelled."; exit 1;;
esac
echo ""

# Remove old venv if exists
if [ -d ~/ros2_servo_venv ]; then
    echo "Removing old virtual environment..."
    rm -rf ~/ros2_servo_venv
    echo "✓ Old venv removed"
fi

# Create new virtual environment
echo "Creating new virtual environment..."
python3 -m venv ~/ros2_servo_venv

# Activate venv
source ~/ros2_servo_venv/bin/activate

# Upgrade pip
echo "Upgrading pip..."
pip install --upgrade pip

# Install required packages
echo "Installing Adafruit libraries..."
pip install adafruit-blinka adafruit-circuitpython-pca9685

echo ""
echo "✓ Virtual environment recreated successfully!"
echo ""
echo "Location: ~/ros2_servo_venv"
echo ""
echo "To use it:"
echo "  source ~/ros2_servo_venv/bin/activate"
echo ""
