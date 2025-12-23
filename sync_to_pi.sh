#!/bin/bash

# ========================================
# Sync Project from Mac to Raspberry Pi
# ========================================

# CONFIGURATION
PI_IP="192.168.1.156"
PI_USER="pranav"

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "========================================="
echo "Syncing Project to Raspberry Pi"
echo "========================================="
echo ""
echo "Source:      $SCRIPT_DIR"
echo "Destination: $PI_USER@$PI_IP:~/pitest1/"
echo ""

# Test connection
echo "Testing connection to Pi..."
if ! ping -c 1 -W 2 $PI_IP > /dev/null 2>&1; then
    echo "⚠️  Warning: Cannot ping $PI_IP"
    read -p "Continue anyway? (y/n): " choice
    if [ "$choice" != "y" ] && [ "$choice" != "Y" ]; then
        echo "Sync cancelled."
        exit 1
    fi
fi

echo ""
echo "Syncing files..."
echo ""

# First, ensure the directory exists on Pi
ssh "$PI_USER@$PI_IP" "mkdir -p ~/"

# Sync files - note the trailing slashes are critical!
rsync -avz --delete \
    --exclude='.git' \
    --exclude='.DS_Store' \
    --exclude='__pycache__' \
    --exclude='*.pyc' \
    --exclude='.vscode' \
    --exclude='ros2_servo_venv' \
    "$SCRIPT_DIR/" \
    "$PI_USER@$PI_IP:~/pitest1/"

if [ $? -eq 0 ]; then
    echo ""
    echo "========================================="
    echo "✓ Sync complete!"
    echo "========================================="
    echo ""
    echo "Verifying files on Pi..."
    ssh "$PI_USER@$PI_IP" "ls -la ~/pitest1/"
    echo ""
    echo "To run on your Pi:"
    echo "  ssh $PI_USER@$PI_IP"
    echo "  cd ~/pitest1"
    echo "  source /opt/ros/jazzy/setup.bash"
    echo "  source ~/ros2_servo_venv/bin/activate"
    echo "  python3 servo_control.py"
else
    echo ""
    echo "❌ Sync failed!"
    exit 1
fi
