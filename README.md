# ROS2 Servo Control for Raspberry Pi 5

Control servos using ROS2 Jazzy on Raspberry Pi 5 with PCA9685 PWM driver.

## Hardware Requirements

- Raspberry Pi 5 (Ubuntu Server 24.04 LTS)
- PCA9685 16-Channel PWM/Servo Driver
- Standard hobby servo (SG90 or similar)
- External 5V power supply for servos

## Wiring

### I2C Connection (Pi ↔ PCA9685)
```
Pi GPIO 3 (SDA)  →  PCA9685 SDA
Pi GPIO 5 (SCL)  →  PCA9685 SCL
Pi GND           →  PCA9685 GND
Pi 3.3V          →  PCA9685 VCC
```

### Servo Connection (PCA9685 → Servo)
```
PCA9685 Channel 0 PWM  →  Servo Signal (Orange/Yellow)
PCA9685 GND            →  Servo Ground (Brown/Black)
External 5V+           →  Servo Power (Red)
External 5V-           →  PCA9685 V+ GND
```

⚠️ **Important:** Connect servo power to external 5V supply, NOT the Pi!

## Quick Start

### First Time Setup (New Pi)

1. **Clone this repository:**
```bash
cd ~
git clone <your-repo-url> ROS2_ServoTest
cd ROS2_ServoTest
```

2. **Run initial setup:**
```bash
chmod +x initial_pi_setup.sh
./initial_pi_setup.sh
```

**What the script does:**
- Updates system packages
- Installs ROS2 Jazzy
- Configures ROS2 in bashrc
- Installs system dependencies (i2c-tools, python3-dev)
- Creates Python virtual environment with Adafruit libraries
- Enables I2C hardware
- Enables GPIO and SPI
- Adds user to hardware groups (i2c, spi, gpio, dialout)
- Scans for I2C devices
- Checks for PCA9685 at address 0x40
- Verifies ROS2 installation

3. **Reboot if prompted:**
```bash
sudo reboot
```

4. **Verify setup after reboot:**
```bash
# Check I2C (should show 40 if PCA9685 connected)
sudo i2cdetect -y 1

# Check user groups (should include i2c, spi, gpio, dialout)
groups $USER
```

### Running Servo Control

**Terminal 1 - Start the servo node:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_servo_venv/bin/activate
cd ~/ROS2_ServoTest
python3 servo_control.py
```

**Terminal 2 - Send commands:**
```bash
source /opt/ros/jazzy/setup.bash

# Move to 90° (middle)
ros2 topic pub /servo_command std_msgs/msg/Float32 "data: 90.0"

# Move to 0° (left)
ros2 topic pub /servo_command std_msgs/msg/Float32 "data: 0.0"

# Move to 180° (right)
ros2 topic pub /servo_command std_msgs/msg/Float32 "data: 180.0"
```

## Files

- `initial_pi_setup.sh` - One-time setup script for new Pi
- `recreate_venv.sh` - Recreate Python virtual environment (rarely needed)
- `servo_control.py` - Main ROS2 servo control node
- `test_servo.py` - Standalone servo test (no ROS2)

## Troubleshooting

### Recreate Virtual Environment

**Only needed if:**
- Python packages are corrupted or broken
- You get "No module named 'board'" errors despite activating venv
- Package installation failed during initial setup
- You want to update to newer package versions

```bash
chmod +x recreate_venv.sh
./recreate_venv.sh
```

**Note:** You do NOT need to recreate the venv every time you use the system. Just activate it:
```bash
source ~/ros2_servo_venv/bin/activate
```

### Common Issues

#### "No module named 'board'"
**Solution:** Activate virtual environment
```bash
source ~/ros2_servo_venv/bin/activate
```

#### "Permission denied" on I2C
**Solution:** Add user to i2c group and logout/login
```bash
sudo usermod -a -G i2c $USER
```

#### PCA9685 not detected
**Solution:** Check wiring and run:
```bash
sudo i2cdetect -y 1  # Should show 40
```
Verify connections:
- Pi GPIO 3 (SDA) → PCA9685 SDA
- Pi GPIO 5 (SCL) → PCA9685 SCL
- Pi GND → PCA9685 GND
- Pi 3.3V → PCA9685 VCC

#### ROS2 command not found
**Solution:** Source ROS2 setup
```bash
source /opt/ros/jazzy/setup.bash
```

### Check I2C Connection

```bash
sudo i2cdetect -y 1
```
Should show `40` if PCA9685 is connected.

### Check ROS2 Topics

```bash
ros2 topic list
ros2 topic echo /servo_command
```

### Permission Issues

If you get permission errors:
```bash
sudo usermod -a -G i2c,spi,gpio,dialout $USER
```
Then logout and login again.

## Maintenance

### Update Python Packages
```bash
source ~/ros2_servo_venv/bin/activate
pip install --upgrade adafruit-blinka adafruit-circuitpython-pca9685
```

### Update ROS2
```bash
sudo apt update
sudo apt upgrade ros-jazzy-desktop
```

## Hardware Groups

Your user should be in these groups:
- `i2c` - I2C device access
- `spi` - SPI device access
- `gpio` - GPIO pin access
- `dialout` - Serial port access

Check with: `groups $USER`

## Virtual Environment

- **Location:** `~/ros2_servo_venv`
- **Activate:** `source ~/ros2_servo_venv/bin/activate`
- **Deactivate:** `deactivate`

## System Configuration

### I2C Pins
- GPIO 3 (Pin 5) - SDA
- GPIO 5 (Pin 3) - SCL

### SPI Pins
- GPIO 10 (Pin 19) - MOSI
- GPIO 9 (Pin 21) - MISO
- GPIO 11 (Pin 23) - SCLK

### Config File
I2C and SPI enabled in `/boot/firmware/config.txt`

## Next Steps

1. Modify `servo_control.py` to add more servos (PCA9685 has 16 channels)
2. Create ROS2 launch files for multi-node systems
3. Add sensor integration
4. Build custom ROS2 messages for complex servo commands
