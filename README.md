# ROS2 Servo Control Test

Simple ROS2 project to control a servo motor using PCA9685 PWM driver on Raspberry Pi 5.

## Hardware Requirements

- Raspberry Pi 5
- PCA9685 PWM/Servo Driver Board
- Servo motor (connected to channel 0)
- Power supply for servos (5-6V)

## Wiring

```
PCA9685 -> Raspberry Pi 5
VCC     -> 3.3V (Pin 1)
GND     -> GND (Pin 6)
SDA     -> GPIO 2 (Pin 3)
SCL     -> GPIO 3 (Pin 5)

Servo -> PCA9685
Signal  -> Channel 0
VCC     -> V+ (External 5-6V)
GND     -> GND
```

## Project Structure

```
ROS2_ServoTest/
├── src/
│   └── servo_control_pkg/
│       ├── servo_control_pkg/
│       │   ├── __init__.py
│       │   └── servo_node.py        # ROS2 servo control node
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
├── test_servo.py                     # Standalone servo test (no ROS2)
├── requirements.txt                  # Python dependencies
└── README.md
```

## Setup Instructions

### 1. Install System Dependencies

```bash
# Enable I2C
sudo raspi-config
# Navigate to: Interface Options -> I2C -> Enable

# Install ROS2 (if not already installed)
# Follow: https://docs.ros.org/en/jazzy/Installation/Ubuntu.html

# Install Python I2C libraries
sudo apt update
sudo apt install -y python3-pip python3-venv i2c-tools
```

### 2. Install Adafruit Libraries

```bash
pip install -r requirements.txt
```

Or install system-wide:
```bash
sudo apt install python3-adafruit-blinka python3-adafruit-circuitpython-pca9685
```

### 3. Build ROS2 Workspace

```bash
cd /Users/pranav/PI5_Projects/ROS2_ServoTest
colcon build
source install/setup.bash
```

## Usage

### Quick Hardware Test (No ROS2)

Test the servo directly without ROS2:

```bash
python3 test_servo.py
```

This will move the servo through several positions to verify hardware connection.

### Run ROS2 Node

```bash
# Source ROS2 and workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Run the servo control node
ros2 run servo_control_pkg servo_node
```

### Control Servo via ROS2 Topics

In another terminal:

```bash
# Source ROS2
source /opt/ros/jazzy/setup.bash

# Move to specific angle (0-180)
ros2 topic pub --once /servo_angle std_msgs/msg/Int32 "{data: 90}"

# Check current angle
ros2 topic echo /servo_angle
```

## Troubleshooting

### I2C Not Detected

```bash
# Check if I2C is enabled
ls /dev/i2c*

# Scan for devices (PCA9685 should show at 0x40)
i2cdetect -y 1
```

### Permission Denied

```bash
# Add user to i2c group
sudo usermod -a -G i2c $USER
# Log out and back in
```

### Servo Not Moving

1. Check power supply to PCA9685 V+ pin (needs 5-6V for servos)
2. Verify servo is connected to channel 0
3. Run `test_servo.py` to isolate ROS2 from hardware issues

### Build Errors

```bash
# Clean and rebuild
rm -rf build/ install/ log/
colcon build
```

## Development

### Rebuild After Code Changes

```bash
colcon build --packages-select servo_control_pkg
source install/setup.bash
```

### View ROS2 Logs

```bash
ros2 run servo_control_pkg servo_node --ros-args --log-level debug
```

## License

MIT
