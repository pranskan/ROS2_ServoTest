#!/bin/bash

echo "========================================="
echo "Setting up Python Virtual Environment"
echo "========================================="
echo ""

# Remove old venv if it exists
if [ -d "ros2_servo_venv" ]; then
    echo "Removing old virtual environment..."
    rm -rf ros2_servo_venv
fi

# Create new venv
echo "Creating new virtual environment..."
python3 -m venv ros2_servo_venv

# Activate venv
echo "Activating virtual environment..."
source ros2_servo_venv/bin/activate

# Upgrade pip
echo "Upgrading pip..."
pip install --upgrade pip

# Install requirements
echo "Installing requirements..."
pip install -r requirements.txt

echo ""
echo "========================================="
echo "✓ Virtual environment ready!"
echo "========================================="
echo ""
echo "To activate in future sessions:"
echo "  source ros2_servo_venv/bin/activate"
echo ""
echo "To deactivate:"
echo "  deactivate"
