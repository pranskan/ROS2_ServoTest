#!/bin/bash

# Function to ask for confirmation
confirm_continue() {
    echo ""
    read -p "Continue? (y/n): " choice
    case "$choice" in 
        y|Y ) echo "Continuing...";;
        n|N ) echo "Setup cancelled."; exit 1;;
        * ) echo "Invalid choice. Setup cancelled."; exit 1;;
    esac
    echo ""
}

echo "======================================================"
echo "Raspberry Pi 5 Initial Setup"
echo "ROS2 Jazzy + Ubuntu Server 24.04 LTS"
echo "6-DOF Robotic Arm Control"
echo "======================================================"
echo ""
echo "This script will:"
echo "  - Install ROS2 Jazzy"
echo "  - Configure I2C, SPI, GPIO"
echo "  - Install Python libraries (Adafruit)"
echo "  - Setup virtual environment"
echo "  - Configure hardware permissions"
echo ""
echo "⚠️  Run this ONCE on a fresh Pi setup"
echo ""
confirm_continue

REBOOT_NEEDED=false

# Step 1: System update
echo "Step 1: Updating system packages..."
confirm_continue
sudo apt update && sudo apt upgrade -y
echo "✓ System updated"
confirm_continue

# Step 2: Install ROS2 Jazzy
echo "Step 2: Installing ROS2 Jazzy..."
echo "This will take several minutes..."
confirm_continue

# Add ROS2 repository
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2
sudo apt update
sudo apt install -y ros-jazzy-ros-base python3-argcomplete
sudo apt install -y ros-dev-tools

echo "✓ ROS2 Jazzy installed"
confirm_continue

# Step 3: Setup ROS2 environment
echo "Step 3: Configuring ROS2 environment..."
confirm_continue
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source /opt/ros/jazzy/setup.bash
echo "✓ ROS2 environment configured"
confirm_continue

# Step 4: Install I2C tools
echo "Step 4: Installing I2C tools..."
confirm_continue
sudo apt install -y i2c-tools python3-smbus
echo "✓ I2C tools installed"
confirm_continue

# Step 5: Install Python packages
echo "Step 5: Installing Python packages..."
echo "Creating virtual environment with ROS2 access..."
confirm_continue

# Remove old venv if exists
if [ -d ~/ros2_servo_venv ]; then
    echo "Removing old virtual environment..."
    rm -rf ~/ros2_servo_venv
fi

# Create new virtual environment with system site packages (for ROS2 access)
echo "Creating virtual environment..."
python3 -m venv --system-site-packages ~/ros2_servo_venv
source ~/ros2_servo_venv/bin/activate

# Upgrade pip
pip install --upgrade pip

# Install required packages from requirements.txt
pip install -r ~/ROS2_ServoTest/requirements.txt

deactivate

echo "✓ Virtual environment created at ~/ros2_servo_venv"
echo "✓ Python packages installed from requirements.txt"
confirm_continue

# Step 6: Enable I2C
echo "Step 6: Enabling I2C..."
confirm_continue
if ! grep -q "^dtparam=i2c_arm=on" /boot/firmware/config.txt; then
    echo "dtparam=i2c_arm=on" | sudo tee -a /boot/firmware/config.txt
    echo "⚠️  I2C enabled in config.txt. REBOOT required!"
    REBOOT_NEEDED=true
else
    echo "I2C already enabled in config.txt"
fi
sudo modprobe i2c-dev i2c-bcm2835
echo "✓ I2C configured"
confirm_continue

# Step 7: Enable SPI
echo "Step 7: Enabling SPI..."
confirm_continue
if ! grep -q "^dtparam=spi=on" /boot/firmware/config.txt; then
    echo "dtparam=spi=on" | sudo tee -a /boot/firmware/config.txt
    echo "⚠️  SPI enabled in config.txt. REBOOT required!"
    REBOOT_NEEDED=true
else
    echo "SPI already enabled in config.txt"
fi
echo "✓ SPI configured"
confirm_continue

# Step 8: Add user to hardware groups
echo "Step 8: Adding user to hardware access groups..."
confirm_continue
sudo usermod -a -G i2c,spi,gpio,dialout $USER
echo "✓ User added to: i2c, spi, gpio, dialout groups"
echo "⚠️  Log out and back in for group changes to take effect"
confirm_continue

# Step 9: Verify I2C devices
echo "Step 9: Scanning for I2C devices..."
echo "⚠️  Connect your PCA9685 board now if not already connected"
confirm_continue
echo "Running I2C scan..."
sudo i2cdetect -y 1
echo ""

if sudo i2cdetect -y 1 | grep -q " 40"; then
    echo "✓ PCA9685 detected at address 0x40"
else
    echo "⚠️  PCA9685 NOT detected. Check wiring:"
    echo "   PCA9685 → Raspberry Pi 5"
    echo "   VCC → Pin 1 (3.3V)"
    echo "   GND → Pin 6 (GND)"
    echo "   SDA → Pin 3 (GPIO 2)"
    echo "   SCL → Pin 5 (GPIO 3)"
fi
echo ""
confirm_continue

# Step 10: Verify ROS2
echo "Step 10: Verifying ROS2 installation..."
confirm_continue
source /opt/ros/jazzy/setup.bash
ros2 --version
echo ""
confirm_continue

# Completion message
echo "======================================================"
echo "✓ SETUP COMPLETE!"
echo "======================================================"
echo ""

if [ "$REBOOT_NEEDED" = true ]; then
    echo "⚠️  REBOOT REQUIRED for I2C/SPI changes!"
    echo ""
    echo "Run: sudo reboot"
    echo ""
    echo "After reboot, continue with the usage instructions."
else
    echo "✓ No reboot required"
fi

echo ""
echo "======================================================"
echo "NEXT STEPS"
echo "======================================================"
echo ""
echo "1. Test hardware (no ROS2 needed):"
echo "   cd ~/ROS2_ServoTest"
echo "   source ~/ros2_servo_venv/bin/activate"
echo "   python3 test_arm.py"
echo ""
echo "2. Use with ROS2:"
echo "   Terminal 1:"
echo "     source /opt/ros/jazzy/setup.bash"
echo "     source ~/ros2_servo_venv/bin/activate"
echo "     python3 servo_control.py"
echo ""
echo "   Terminal 2:"
echo "     source /opt/ros/jazzy/setup.bash"
echo "     ros2 topic pub --once /arm_demo std_msgs/msg/String \"data: 'demo'\""
echo ""
echo "3. Calibrate servos (if needed):"
echo "   python3 calibrate_servo.py"
echo ""
echo "======================================================"
echo ""
