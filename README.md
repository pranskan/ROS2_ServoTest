# ROS2 Robotic Arm Control

ROS2 project to control a 6-DOF robotic arm using PCA9685 PWM driver on Raspberry Pi 5.

## Hardware Requirements

- Raspberry Pi 5
- PCA9685 PWM/Servo Driver Board
- 6x Servo motors (MG996R or similar)
- Power supply for servos (5-6V, 10A+ recommended for 6 servos)

## Wiring

```
PCA9685 -> Raspberry Pi 5
VCC     -> 3.3V (Pin 1)
GND     -> GND (Pin 6)
SDA     -> GPIO 2 (Pin 3)
SCL     -> GPIO 3 (Pin 5)

Servos -> PCA9685 (YOUR ACTUAL WIRING)
Servo (Gripper)     -> Channel 0
Servo (Wrist Roll)  -> Channel 1
Servo (Wrist Pitch) -> Channel 2
Servo (Elbow)       -> Channel 3
Servo (Shoulder)    -> Channel 4
Servo (Base)        -> Channel 5

All Servo Power:
Red (Power)  -> PCA9685 V+ (External 5-6V, 10A)
Black (GND)  -> PCA9685 GND
```

**⚠️ Important:** 6 servos can draw 10-12A total. Use adequate power supply!

## Project Structure

```
ROS2_RoboticArm/
├── src/
│   └── arm_control_pkg/
│       ├── arm_control_pkg/
│       │   ├── __init__.py
│       │   └── arm_node.py        # ROS2 robotic arm control node
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
├── test_arm.py                     # Standalone arm test (no ROS2)
├── requirements.txt                # Python dependencies
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
cd /Users/pranav/PI5_Projects/ROS2_RoboticArm
colcon build
source install/setup.bash
```

## Usage

### Quick Hardware Test (No ROS2)

Test all 6 servos:

```bash
python3 test_arm.py
```

### Run ROS2 Node

```bash
# Terminal 1: Start the arm controller
source /opt/ros/jazzy/setup.bash
source ~/ros2_servo_venv/bin/activate
python3 servo_control.py
```

### Control Robotic Arm via ROS2

```bash
# Terminal 2: Send commands
source /opt/ros/jazzy/setup.bash

# Move all servos to center (90°)
ros2 topic pub --once /arm_command std_msgs/msg/Float32MultiArray "data: [90.0, 90.0, 90.0, 90.0, 90.0, 90.0]"

# Example positions:
# [base, shoulder, elbow, wrist_pitch, wrist_roll, gripper]

# Reach forward position
ros2 topic pub --once /arm_command std_msgs/msg/Float32MultiArray "data: [90.0, 45.0, 45.0, 90.0, 90.0, 90.0]"

# Reach up position
ros2 topic pub --once /arm_command std_msgs/msg/Float32MultiArray "data: [90.0, 135.0, 135.0, 90.0, 90.0, 90.0]"

# Close gripper
ros2 topic pub --once /arm_command std_msgs/msg/Float32MultiArray "data: [90.0, 90.0, 90.0, 90.0, 90.0, 0.0]"

# Open gripper
ros2 topic pub --once /arm_command std_msgs/msg/Float32MultiArray "data: [90.0, 90.0, 90.0, 90.0, 90.0, 180.0]"
```

## Servo Mapping

| Channel | Joint | Function | Range |
|---------|-------|----------|-------|
| 5 | Base | Rotation | 0-180° |
| 4 | Shoulder | Up/Down | 0-180° |
| 3 | Elbow | Bend | 0-180° |
| 2 | Wrist Pitch | Tilt | 0-180° |
| 1 | Wrist Roll | Rotate | 0-180° |
| 0 | Gripper | Open/Close | 0-180° |

**Note:** ROS2 messages use logical order [base, shoulder, elbow, wrist_pitch, wrist_roll, gripper] which is automatically mapped to the correct channels.

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
2. Verify servos are connected to correct channels
3. Run `test_arm.py` to isolate ROS2 from hardware issues

### Build Errors

```bash
# Clean and rebuild
rm -rf build/ install/ log/
colcon build
```

## Development

### Rebuild After Code Changes

```bash
colcon build --packages-select arm_control_pkg
source install/setup.bash
```

### View ROS2 Logs

```bash
ros2 run arm_control_pkg arm_node --ros-args --log-level debug
```

## License

MIT
