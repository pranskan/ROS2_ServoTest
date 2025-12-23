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

echo "=== Raspberry Pi 5 Servo + ROS2 Jazzy Setup ==="
echo "=== Ubuntu Server 24.04 LTS ==="
echo ""
echo "This script will install ROS2 Jazzy and configure your Pi for servo control."
confirm_continue

# Step 1: Update system
echo "Step 1: Updating system packages..."
echo "This may take several minutes."
confirm_continue
sudo apt update
sudo apt upgrade -y
echo "✓ System updated"
confirm_continue

# Step 2: Install ROS2 Jazzy
echo "Step 2: Installing ROS2 Jazzy..."
echo "This is a large download and will take 10-20 minutes."
confirm_continue
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
sudo apt update
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop python3-pip python3-venv
echo "✓ ROS2 Jazzy installed"
confirm_continue

# Step 3: Configure bash for ROS2
echo "Step 3: Configuring ROS2 in bashrc..."
confirm_continue
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi
source /opt/ros/jazzy/setup.bash
echo "✓ ROS2 configured"
confirm_continue

# Step 4: Install system dependencies for Python packages
echo "Step 4: Installing system dependencies..."
confirm_continue
sudo apt install -y python3-dev i2c-tools
echo "✓ System dependencies installed"
confirm_continue

# Step 5: Create Python virtual environment
echo "Step 5: Creating Python virtual environment..."
confirm_continue
python3 -m venv ~/ros2_servo_venv
source ~/ros2_servo_venv/bin/activate
pip install --upgrade pip
echo "✓ Virtual environment created at ~/ros2_servo_venv"
confirm_continue

# Step 6: Install Python dependencies in venv
echo "Step 6: Installing Python packages in venv..."
confirm_continue
pip install adafruit-blinka adafruit-circuitpython-pca9685
echo "✓ Python dependencies installed in venv"
confirm_continue

# Step 7: Enable I2C
echo "Step 7: Enabling I2C..."
confirm_continue
if ! grep -q "^dtparam=i2c_arm=on" /boot/firmware/config.txt; then
    echo "dtparam=i2c_arm=on" | sudo tee -a /boot/firmware/config.txt
    echo "⚠️  I2C enabled. REBOOT required!"
    REBOOT_NEEDED=true
fi
sudo modprobe i2c-dev i2c-bcm2835
echo "✓ I2C configured"
confirm_continue

# Step 8: Enable GPIO and SPI
echo "Step 8: Enabling GPIO and SPI..."
confirm_continue
if ! grep -q "^dtparam=spi=on" /boot/firmware/config.txt; then
    echo "dtparam=spi=on" | sudo tee -a /boot/firmware/config.txt
    echo "⚠️  SPI enabled. REBOOT required!"
    REBOOT_NEEDED=true
fi
# Load GPIO modules
sudo modprobe gpio-mockup 2>/dev/null || true
echo "✓ GPIO and SPI configured"
confirm_continue

# Step 9: Add user to hardware groups
echo "Step 9: Adding user to hardware access groups..."
confirm_continue
sudo usermod -a -G i2c,spi,gpio,dialout $USER
echo "✓ User added to i2c, spi, gpio, dialout groups"
confirm_continue

# Step 10: Check I2C devices
echo "Step 10: Scanning I2C devices..."
echo "Connect your PCA9685 board now if not already connected."
confirm_continue
sudo i2cdetect -y 1
echo ""
confirm_continue

# Step 11: Check for PCA9685
echo "Step 11: Checking for PCA9685..."
if sudo i2cdetect -y 1 | grep -q "40"; then
    echo "✓ PCA9685 detected at address 0x40"
else
    echo "⚠️  PCA9685 NOT found. Check wiring:"
    echo "   - Pi GPIO 3 (SDA) → PCA9685 SDA"
    echo "   - Pi GPIO 5 (SCL) → PCA9685 SCL"
    echo "   - Pi GND → PCA9685 GND"
    echo "   - Pi 3.3V → PCA9685 VCC"
fi
echo ""
confirm_continue

# Step 12: Verify ROS2
echo "Step 12: Verifying ROS2 installation..."
confirm_continue
source /opt/ros/jazzy/setup.bash
ros2 --version
echo ""

echo "====================================="
echo "✓ Setup complete!"
echo "====================================="
echo ""

if [ "$REBOOT_NEEDED" = true ]; then
    echo "⚠️  REBOOT REQUIRED!"
    echo ""
    echo "Run: sudo reboot"
    echo ""
    echo "After reboot, continue to next steps below."
    echo ""
fi

echo "====================================="
echo "HOW TO USE ROS2 SERVO CONTROL"
echo "====================================="
echo ""
echo "STEP 1: Open Terminal 1"
echo "----------------------------------------"
echo "source /opt/ros/jazzy/setup.bash"
echo "source ~/ros2_servo_venv/bin/activate"
echo "cd ~/ROS2_ServoTest"
echo "python3 servo_control.py"
echo ""
echo "You should see: [INFO] Servo node started"
echo ""
echo ""
echo "STEP 2: Open Terminal 2 (new window)"
echo "----------------------------------------"
echo "source /opt/ros/jazzy/setup.bash"
echo ""
echo "Then send servo commands:"
echo ""
echo "# Move to 90° (middle)"
echo "ros2 topic pub /servo_command std_msgs/msg/Float32 \"data: 90.0\""
echo ""
echo "# Move to 0° (left)"
echo "ros2 topic pub /servo_command std_msgs/msg/Float32 \"data: 0.0\""
echo ""
echo "# Move to 180° (right)"
echo "ros2 topic pub /servo_command std_msgs/msg/Float32 \"data: 180.0\""
echo ""
echo ""
echo "HARDWARE ACCESS:"
echo "----------------------------------------"
echo "User groups: i2c, spi, gpio, dialout"
echo "I2C enabled: GPIO 3 (SDA), GPIO 5 (SCL)"
echo "SPI enabled: GPIO 10 (MOSI), GPIO 9 (MISO), GPIO 11 (SCLK)"
echo "GPIO: All pins accessible"
echo ""
echo "VIRTUAL ENVIRONMENT:"
echo "----------------------------------------"
echo "Location: ~/ros2_servo_venv"
echo "Activate:  source ~/ros2_servo_venv/bin/activate"
echo "Deactivate: deactivate"
echo ""
