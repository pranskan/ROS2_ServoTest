# ROS2 Robotic Arm Control

Simple ROS2 control for 6-DOF robotic arm using PCA9685 on Raspberry Pi 5.

## Files

```
ROS2_ServoTest/
├── servo_control.py           # Main ROS2 node (with safe startup)
├── teleop_keyboard.py         # Keyboard control (MAIN METHOD)
└── README.md                  # This file
```

## Hardware

- Raspberry Pi 5 (Ubuntu 24.04)
- PCA9685 PWM Driver
- Servos:
  - 6x MG996R (180° servos): Gripper, Wrist Roll, Wrist Pitch, Elbow, Shoulder, Base
- 5-6V 10A power supply

## Servo Types

| Channel | Name | Type | Range | Center |
|---------|------|------|-------|--------|
| 0 | Gripper | 180° | 0-180° | 90° |
| 1 | Wrist Roll | 180° | 0-180° | 90° |
| 2 | Wrist Pitch | 180° | 0-180° | 90° |
| 3 | Elbow | 180° | 0-180° | 90° |
| 4 | Shoulder | 180° | 0-180° | 90° |
| 5 | Base | 180° | 0-180° | 90° |

## Wiring

```
PCA9685 -> Pi 5:
  VCC -> 3.3V (Pin 1)
  GND -> GND (Pin 6)
  SDA -> GPIO 2 (Pin 3)
  SCL -> GPIO 3 (Pin 5)

Servos -> PCA9685:
  Ch 0: Gripper (MG996R)
  Ch 1: Wrist Roll (MG996R)
  Ch 2: Wrist Pitch (MG996R)
  Ch 3: Elbow (MG996R)
  Ch 4: Shoulder (MG996R)
  Ch 5: Base (MG996R)

Servo Power:
  Red -> PCA9685 V+ (5-6V, 10A)
  Black -> PCA9685 GND
```

## Setup

### 1. Install ROS2 Jazzy

```bash
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt install -y curl gnupg lsb-release

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

sudo apt update
sudo apt install -y ros-jazzy-desktop

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Enable I2C

```bash
sudo raspi-config
# Interface Options -> I2C -> Enable
sudo reboot

# Verify
ls /dev/i2c*  # Should show /dev/i2c-1
sudo i2cdetect -y 1  # Should show '40' at 0x40
```

### 3. Install Python Dependencies

```bash
cd ~/ROS2_ServoTest

# Create virtual environment with system packages
python3 -m venv --system-site-packages ~/ros2_servo_venv
source ~/ros2_servo_venv/bin/activate

# Install libraries
pip3 install adafruit-blinka adafruit-circuitpython-pca9685 numpy

# Add user to i2c group
sudo usermod -a -G i2c $USER
# Log out and back in
```

## Usage

### Start the Arm

```bash
cd ~/ROS2_ServoTest
source /opt/ros/jazzy/setup.bash
source ~/ros2_servo_venv/bin/activate

# Start main node (includes safe initialization)
python3 servo_control.py
```

**Wait for "READY!" message (~15 seconds)**

### Keyboard Control (RECOMMENDED)

In another terminal:

```bash
cd ~/ROS2_ServoTest
source /opt/ros/jazzy/setup.bash
source ~/ros2_servo_venv/bin/activate

python3 teleop_keyboard.py
```

**Controls:**
- `0-5`: Select motor
- `+/-`: Move by 5°
- `]/[`: Move by 1°
- `c`: Center all motors
- `s`: Show status with XYZ position
- `f/n/m`: Fast (10°) / Normal (5°) / Slow (1°) mode
- `h`: Show help
- `q`: Quit

### ROS2 Direct Commands

```bash
source /opt/ros/jazzy/setup.bash

# Move to center
ros2 topic pub --once /arm_demo std_msgs/msg/String "data: 'center'"

# Direct motor control (all servos at 90°)
ros2 topic pub --once /arm_command std_msgs/msg/Float32MultiArray \
  "data: [90.0, 90.0, 90.0, 90.0, 90.0, 90.0]"
```

## ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/arm_command` | Float32MultiArray | Direct angles [6 values] |
| `/arm_demo` | String | Commands: 'center' |

## Configuration

### Adjust Movement Speed

Edit `servo_control.py`:

```python
self.movement_speed = 2.0  # degrees/step (lower = smoother)
self.step_delay = 0.02     # seconds (lower = faster)
```

## Troubleshooting

### Servos Jittery on Startup

Normal - safe initialization takes ~15 seconds.

### I2C Not Found

```bash
sudo apt install -y i2c-tools
sudo i2cdetect -y 1
```

Should show `40` at address 0x40.

### Permission Denied

```bash
sudo usermod -a -G i2c $USER
# Log out and log back in
```

### Servo Not Moving

- Check if angle is within servo's physical limits
- Verify power supply is providing 5-6V at 10A

## Quick Reference

```bash
# Daily workflow:
cd ~/ROS2_ServoTest
source /opt/ros/jazzy/setup.bash
source ~/ros2_servo_venv/bin/activate

# Terminal 1:
python3 servo_control.py     # Wait for "READY!"

# Terminal 2:
python3 teleop_keyboard.py   # Control with keyboard
```

## License

MIT
