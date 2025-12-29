# ROS2 Robotic Arm Control

ROS2 Jazzy project to control a 6-DOF robotic arm using PCA9685 PWM driver on Raspberry Pi 5.

## Hardware Requirements

- Raspberry Pi 5 (Ubuntu Server 24.04 LTS)
- PCA9685 PWM/Servo Driver Board
- 6x Servo motors:
  - Channels 0-1: MG996R (Gripper, Wrist Roll)
  - Channels 2-5: DS3218 (Wrist Pitch, Elbow, Shoulder, Base)
- External power supply for servos (5-6V, 10A+ recommended)

## Wiring

```
PCA9685 -> Raspberry Pi 5
VCC     -> 3.3V (Pin 1)
GND     -> GND (Pin 6)
SDA     -> GPIO 2 (Pin 3)
SCL     -> GPIO 3 (Pin 5)

Servos -> PCA9685 Channels
Channel 0: Gripper (MG996R)
Channel 1: Wrist Roll (MG996R)
Channel 2: Wrist Pitch (DS3218)
Channel 3: Elbow (DS3218)
Channel 4: Shoulder (DS3218)
Channel 5: Base (DS3218)

All Servo Power:
Red (Power)  -> PCA9685 V+ (External 5-6V, 10A)
Black (GND)  -> PCA9685 GND
```

**⚠️ Important:** 6 servos can draw 10-12A total. Use adequate power supply!

## Project Structure

```
ROS2_ServoTest/
├── servo_control.py          # Main ROS2 control node
├── test_arm.py               # Interactive testing tool (no ROS2)
├── kinematics.py             # Inverse kinematics calculations
├── calibrate_servo.py        # Servo calibration tool
├── initial_pi_setup.sh       # Initial Pi setup script
├── requirements.txt          # Python dependencies
└── README.md                 # This file
```

## Initial Setup (Run Once)

### Option 1: Automated Setup Script

```bash
cd ~/ROS2_ServoTest
chmod +x initial_pi_setup.sh
./initial_pi_setup.sh
```

Follow the prompts. The script will:
- Install ROS2 Jazzy
- Configure I2C/SPI/GPIO
- Create virtual environment
- Install Python libraries

### Option 2: Manual Setup

See initial_pi_setup.sh for detailed commands.

## Usage

### Interactive Testing (Recommended First)

```bash
cd ~/ROS2_ServoTest
source ~/ros2_servo_venv/bin/activate
python3 test_arm.py
```

**Available commands:**
- `5 90` - Move motor 5 to 90°
- `all 90` - Move all motors to 90°
- `xyz 20 0 30` - Move to XYZ position (cm)
- `center` - Return all to 90°
- `status` - Show current positions
- `reset` - Disable all PWM
- `quit` - Exit

### ROS2 Node

**Terminal 1 - Start node:**
```bash
cd ~/ROS2_ServoTest
source /opt/ros/jazzy/setup.bash
source ~/ros2_servo_venv/bin/activate
python3 servo_control.py
```

**Terminal 2 - Send commands:**
```bash
source /opt/ros/jazzy/setup.bash

# Run demo sequence (tests all motors)
ros2 topic pub --once /arm_demo std_msgs/msg/String "data: 'demo'"

# Sweep individual motor (channel 5 = base)
ros2 topic pub --once /arm_demo std_msgs/msg/String "data: 'sweep:5'"

# Move to XYZ position (20cm forward, 0 left/right, 30cm up)
ros2 topic pub --once /arm_xyz geometry_msgs/msg/Point "x: 20.0
y: 0.0
z: 30.0"

# Direct motor control [gripper, wrist_roll, wrist_pitch, elbow, shoulder, base]
ros2 topic pub --once /arm_command std_msgs/msg/Float32MultiArray "data: [90.0, 90.0, 90.0, 90.0, 90.0, 90.0]"

# Return to center
ros2 topic pub --once /arm_demo std_msgs/msg/String "data: 'center'"
```

## ROS2 Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/arm_command` | Float32MultiArray | Direct motor angles [6 values, 0-180°] |
| `/arm_demo` | String | Demo commands: 'demo', 'sweep:N', 'center' |
| `/arm_xyz` | Point | XYZ positioning (uses inverse kinematics) |

## Motor Channel Mapping

| Channel | Joint | Type | Function |
|---------|-------|------|----------|
| 0 | Gripper | MG996R | Open/Close (0°=closed, 180°=open) |
| 1 | Wrist Roll | MG996R | Rotate wrist |
| 2 | Wrist Pitch | DS3218 | Tilt wrist up/down |
| 3 | Elbow | DS3218 | Bend elbow |
| 4 | Shoulder | DS3218 | Lift arm up/down |
| 5 | Base | DS3218 | Rotate base left/right |

## Calibration

If servos don't reach full 0-180° range or make buzzing sounds:

```bash
python3 calibrate_servo.py
```

Update the calibrated values in:
- `test_arm.py` → `SERVO_MIN_MG996R/DS3218`, `SERVO_MAX_MG996R/DS3218`
- `servo_control.py` → `get_pulse_range()` method

## Kinematics Configuration

Measure your robot's link lengths and update in `kinematics.py`:

```python
# Default values - MEASURE YOUR ROBOT!
L1 = 10.0  # Base height (cm)
L2 = 15.0  # Upper arm length (cm)
L3 = 15.0  # Forearm length (cm)
L4 = 10.0  # Gripper length (cm)
```

Test kinematics:
```bash
python3 kinematics.py
```

## Troubleshooting

### I2C Not Detected

```bash
ls /dev/i2c*                    # Should show /dev/i2c-1
sudo i2cdetect -y 1             # Should show '40' at address 0x40
```

If not detected:
```bash
sudo raspi-config
# Interface Options -> I2C -> Enable
sudo reboot
```

### Permission Denied

```bash
sudo usermod -a -G i2c $USER
# Log out and back in
```

### Servo Not Moving

1. Check external power supply (5-6V, 10A+)
2. Verify servo connections to correct channels
3. Run `python3 test_arm.py` to isolate issues
4. Calibrate servos with `python3 calibrate_servo.py`

### Jerky Movement on Startup

This is normal - servos initialize one at a time with 0.2s delays to prevent simultaneous power draw.

## Files Description

| File | Purpose |
|------|---------|
| `servo_control.py` | Main ROS2 node (requires ROS2) |
| `test_arm.py` | Interactive testing (no ROS2 needed) |
| `kinematics.py` | Inverse kinematics math |
| `calibrate_servo.py` | Find optimal pulse ranges |
| `initial_pi_setup.sh` | One-time Pi setup |

## Quick Start Summary

```bash
# 1. Initial setup (once)
./initial_pi_setup.sh

# 2. Test hardware
python3 test_arm.py
# Type: center, status, quit

# 3. Use with ROS2
python3 servo_control.py
# (In another terminal)
ros2 topic pub --once /arm_demo std_msgs/msg/String "data: 'demo'"
```

## License

MIT
