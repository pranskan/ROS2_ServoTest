# Servo Motor Control with ROS2 Jazzy

Raspberry Pi 5 servo control using ROS2 Jazzy on Ubuntu Server 24.04 LTS.

## Requirements

- Raspberry Pi 5
- Ubuntu Server 24.04 LTS
- PCA9685 PWM driver
- Servo motor
- External 5V power supply

## Setup

```bash
chmod +x setup_and_run.sh
./setup_and_run.sh
```

If it says "REBOOT REQUIRED", run:
```bash
sudo reboot
```

## Usage

### Terminal 1: Start ROS2 servo node
```bash
source /opt/ros/jazzy/setup.bash
python3 servo_control.py
```

Output:
```
[INFO] Servo node started (ROS2 Jazzy)
[INFO] Listening on: /servo_command
```

### Terminal 2: Send servo commands
```bash
source /opt/ros/jazzy/setup.bash

# Move to 90° (middle)
ros2 topic pub /servo_command std_msgs/msg/Float32 "data: 90.0"

# Move to 0° (left)
ros2 topic pub /servo_command std_msgs/msg/Float32 "data: 0.0"

# Move to 180° (right)
ros2 topic pub /servo_command std_msgs/msg/Float32 "data: 180.0"

# Move to any angle (0-180)
ros2 topic pub /servo_command std_msgs/msg/Float32 "data: 45.0"
```

## Hardware Wiring

**Pi 5 → PCA9685:**
- GPIO 3 (SDA) → SDA
- GPIO 5 (SCL) → SCL
- GND → GND
- 3.3V → VCC

**PCA9685 → Servo:**
- Channel 0 → Servo signal wire
- GND → Servo ground
- External 5V → Servo power

## Troubleshooting

### Check I2C
```bash
sudo i2cdetect -y 1
```
Should show "40" at address 0x40

### Check ROS2
```bash
ros2 --version
```
Should show: `ros2 release jazzy`

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
