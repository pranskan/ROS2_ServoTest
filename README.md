# Servo Motor Control with ROS2 Jazzy

Raspberry Pi 5 servo control using ROS2 Jazzy on Ubuntu Server 24.04 LTS with Python virtual environment.

## Requirements

- Raspberry Pi 5
- Ubuntu Server 24.04 LTS
- PCA9685 PWM driver
- Servo motor
- External 5V power supply

## Hardware Wiring

**Pi 5 → PCA9685:**
- GPIO 3 (SDA) → SDA
- GPIO 5 (SCL) → SCL
- GND → GND
- 3.3V → VCC

**PCA9685 → Servo:**
- Channel 0 → Servo signal wire
- GND → Servo ground
- External 5V+ → V+ (servo power)

## Quick Setup (Raspberry Pi)

### 1. Run automated setup
```bash
cd ~/ROS2_ServoTest
chmod +x setup_and_run.sh
./setup_and_run.sh
```

The script will:
- Install ROS2 Jazzy
- Configure I2C, SPI, and GPIO
- Create Python virtual environment at `~/ros2_servo_venv`
- Install Adafruit libraries
- Add user to hardware access groups

### 2. Reboot if prompted
```bash
sudo reboot
```

## Usage

### Test Hardware (Without ROS2)

Test your servo hardware before using ROS2:

```bash
cd ~/ROS2_ServoTest
source ~/ros2_servo_venv/bin/activate
python3 test_servo.py
```

This will move the servo through positions: 0° → 90° → 180° → 90°

### Run with ROS2

#### Terminal 1: Start servo node
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_servo_venv/bin/activate
cd ~/ROS2_ServoTest
python3 servo_control.py
```

Expected output:
```
[INFO] Servo node started (ROS2 Jazzy)
[INFO] Listening on: /servo_command
```

#### Terminal 2: Send servo commands
```bash
source /opt/ros/jazzy/setup.bash

# Move to 90° (center)
ros2 topic pub /servo_command std_msgs/msg/Float32 "data: 90.0"

# Move to 0° (left)
ros2 topic pub /servo_command std_msgs/msg/Float32 "data: 0.0"

# Move to 180° (right)
ros2 topic pub /servo_command std_msgs/msg/Float32 "data: 180.0"

# Move to any angle (0-180)
ros2 topic pub /servo_command std_msgs/msg/Float32 "data: 45.0"
```

## Development (Mac → Pi Sync)

### From your Mac:

```bash
cd /Users/pranav/PI5_Projects/ROS2_ServoTest
chmod +x sync_to_pi.sh
./sync_to_pi.sh
```

This syncs all project files to `~/ROS2_ServoTest` on your Pi.

### Recreate virtual environment (if needed):

On Pi:
```bash
cd ~/ROS2_ServoTest
chmod +x setup_venv.sh
./setup_venv.sh
```

Or manually:
```bash
python3 -m venv ~/ros2_servo_venv
source ~/ros2_servo_venv/bin/activate
pip install -r requirements.txt
```

## Troubleshooting

### Check I2C connection
```bash
sudo i2cdetect -y 1
```
Should show `40` at address 0x40 if PCA9685 is connected.

### Check ROS2 version
```bash
ros2 --version
```
Should show: `ros2 cli version: jazzy`

### Check ROS2 topics
```bash
ros2 topic list
```
Should show: `/servo_command`

### Check ROS2 nodes
```bash
ros2 node list
```
Should show: `/servo_node`

### Check user groups
```bash
groups
```
Should include: `i2c spi gpio dialout`

### Check virtual environment
```bash
which python
```
When activated, should show: `/home/pranav/ros2_servo_venv/bin/python`

### I2C not working
1. Check wiring
2. Verify I2C is enabled: `grep i2c_arm /boot/firmware/config.txt`
3. Check I2C modules: `lsmod | grep i2c`
4. Reboot if you just enabled I2C

### Permission denied errors
```bash
# Re-login or reboot after setup to apply group changes
sudo reboot
```

### Virtual environment issues
```bash
# Remove and recreate
rm -rf ~/ros2_servo_venv
./setup_venv.sh
```

## Project Structure

```
ROS2_ServoTest/
├── README.md                 # This file
├── requirements.txt          # Python dependencies
├── servo_control.py          # ROS2 servo node
├── test_servo.py            # Hardware test (no ROS2)
├── setup_and_run.sh         # Complete Pi setup
├── setup_venv.sh            # Quick venv recreation
└── sync_to_pi.sh            # Mac → Pi sync script
```

## Hardware Access Enabled

- **I2C**: GPIO 3 (SDA), GPIO 5 (SCL)
- **SPI**: GPIO 10 (MOSI), GPIO 9 (MISO), GPIO 11 (SCLK)
- **GPIO**: All GPIO pins accessible
- **Groups**: User added to `i2c`, `spi`, `gpio`, `dialout`

## Python Dependencies

Installed in virtual environment at `~/ros2_servo_venv`:
- `adafruit-blinka` - CircuitPython on Linux
- `adafruit-circuitpython-pca9685` - PCA9685 PWM driver

## ROS2 Configuration

- **Distribution**: Jazzy
- **Install Type**: Desktop (via apt)
- **Auto-sourced**: Yes (in ~/.bashrc)
- **Python Bridge**: Native Python3 support

## Tips

1. Always activate the venv before running Python scripts
2. ROS2 commands don't need venv, only Python scripts do
3. Use `test_servo.py` to verify hardware before using ROS2
4. The sync script excludes `.venv` folders to save bandwidth
5. Virtual environment location: `~/ros2_servo_venv` (outside project)

## Common Commands

```bash
# Activate environment
source ~/ros2_servo_venv/bin/activate

# Deactivate environment
deactivate

# Check Python location
which python

# List installed packages
pip list

# Update packages
pip install --upgrade adafruit-blinka adafruit-circuitpython-pca9685

# ROS2 topic echo (monitor commands)
ros2 topic echo /servo_command
```
