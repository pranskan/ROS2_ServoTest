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

### Running Multiple Control Methods Simultaneously

The servo control node can receive commands from multiple sources at the same time through ROS2 topics:

**Terminal 1 - Start servo control node (required):**
```bash
cd ~/ROS2_ServoTest
source /opt/ros/jazzy/setup.bash
source ~/ros2_servo_venv/bin/activate
python3 servo_control.py
```

**Terminal 2 - Keyboard teleoperation (optional):**
```bash
cd ~/ROS2_ServoTest
source /opt/ros/jazzy/setup.bash
source ~/ros2_servo_venv/bin/activate
python3 teleop_keyboard.py
```

**Terminal 3 - Send ROS2 commands (optional):**
```bash
source /opt/ros/jazzy/setup.bash

# Move to XYZ position (with path planning)
ros2 topic pub --once /arm_xyz geometry_msgs/msg/Point "x: 20.0
y: 0.0
z: 30.0"

# Direct motor control
ros2 topic pub --once /arm_command std_msgs/msg/Float32MultiArray "data: [90.0, 90.0, 90.0, 90.0, 90.0, 90.0]"
```

**How it works:**
- All control methods publish to ROS2 topics (`/arm_command`, `/arm_xyz`, etc.)
- `servo_control.py` subscribes to these topics and moves the servos
- Commands are processed in the order they arrive
- Multiple publishers can coexist without conflicts

**Typical workflow:**
1. Start `servo_control.py` (always required)
2. Use `teleop_keyboard.py` for manual exploration and testing
3. Use `ros2 topic pub` or custom scripts for automated sequences
4. Switch between control methods at any time

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

### Keyboard Teleoperation

Control the arm interactively with real-time XYZ position feedback:

```bash
cd ~/ROS2_ServoTest
source /opt/ros/jazzy/setup.bash
source ~/ros2_servo_venv/bin/activate
python3 teleop_keyboard.py
```

**Controls:**
- **0-5**: Select motor (Gripper, Wrist Roll, Wrist Pitch, Elbow, Shoulder, Base)
- **+/=**: Increase angle by current increment
- **-/_**: Decrease angle by current increment
- **]**: Increase by 1°
- **[**: Decrease by 1°
- **c**: Center all motors to 90°
- **s**: Show current status with XYZ position
- **f**: Fast mode (10° increments)
- **n**: Normal mode (5° increments)
- **m**: Slow mode (1° increments)
- **h**: Show help
- **q**: Quit

**Features:**
- Real-time XYZ position display after each movement
- Requires `servo_control.py` running in another terminal
- Non-blocking - can run alongside other control methods

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

# Path planning controls
ros2 topic pub --once /arm_obstacles std_msgs/msg/String "data: 'enable_planning'"
ros2 topic pub --once /arm_obstacles std_msgs/msg/String "data: 'disable_planning'"
```

## ROS2 Topics

| Topic | Message Type | Description | Publishers |
|-------|-------------|-------------|------------|
| `/arm_command` | Float32MultiArray | Direct motor angles [6 values, 0-180°] | teleop_keyboard.py, ros2 CLI |
| `/arm_demo` | String | Demo commands: 'demo', 'sweep:N', 'center' | ros2 CLI |
| `/arm_xyz` | Point | XYZ positioning (uses inverse kinematics) | **path_executor.py**, ros2 CLI, custom scripts |
| `/arm_obstacles` | String | Path planning: 'enable_planning', 'disable_planning', 'add:name:bounds' | ros2 CLI |

**Note:** All publishers can run simultaneously. The `servo_control.py` node processes commands as they arrive.

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

## Path Planning

The `PathPlanner` class creates smooth, collision-free paths between two positions.

### Features

- **Smooth Path Planning**: Interpolates joint angles to ensure smooth motion
- **Linear Path Planning**: Creates straight-line paths in Cartesian space
- **Configurable Step Size**: Control maximum joint angle changes per waypoint
- **Automatic Waypoint Generation**: Calculates optimal number of waypoints

### Using Path Planning with ROS2

**Terminal 1 - Start servo control (required):**
```bash
cd ~/ROS2_ServoTest
source /opt/ros/jazzy/setup.bash
source ~/ros2_servo_venv/bin/activate
python3 servo_control.py
```

**Terminal 2 - Execute a planned path:**
```bash
cd ~/ROS2_ServoTest
source /opt/ros/jazzy/setup.bash
source ~/ros2_servo_venv/bin/activate

# Execute path with default positions
python3 path_executor.py

# Custom start and end positions
python3 path_executor.py --start 0 -31 29 --end -32 12 26

# Smoother motion (smaller steps, slower)
python3 path_executor.py --start 0 -31 29 --end -32 12 26 --max-change 3 --delay 0.3

# Faster motion (larger steps, quicker)
python3 path_executor.py --start 0 -31 29 --end -32 12 26 --max-change 10 --delay 0.1
```

**Path executor arguments:**
- `--start X Y Z`: Starting position in cm (default: 0.0 -31.07 29.15)
- `--end X Y Z`: Target position in cm (default: -31.76 11.56 26.34)
- `--max-change DEGREES`: Max joint angle change per waypoint (default: 5.0°)
- `--delay SECONDS`: Delay between waypoints (default: 0.2s)

**How it works:**
1. `path_executor.py` calculates waypoints using path planner
2. Publishes each waypoint to `/arm_xyz` topic
3. `servo_control.py` receives waypoints and moves servos
4. Waits between waypoints for smooth execution

### Command-Line Testing (No ROS2)

Test the path planner algorithm without ROS2:

```bash
# Run with default test case
python3 path_planner.py

# Specify custom start and end positions
python3 path_planner.py --start 0 -31.07 29.15 --end -31.76 11.56 26.34

# Check if a position is reachable
python3 path_planner.py --check-only --start 0 -31.07 29.15

# Adjust max joint change
python3 path_planner.py --start 10 20 30 --end -10 -20 25 --max-change 3
```

**Note:** `path_planner.py` only tests the algorithm - it doesn't move the robot. Use `path_executor.py` to actually execute paths.

### Usage

```python
from kinematics import ArmKinematics
from path_planner import PathPlanner

# Initialize
kinematics = ArmKinematics()
planner = PathPlanner(kinematics)

# Define start and end positions (in cm)
start_xyz = (-0.00, -31.07, 29.15)
end_xyz = (-31.76, 11.56, 26.34)

# Plan a smooth path (max 5° joint change per step)
waypoints = planner.plan_best_path(start_xyz, end_xyz, max_joint_change=5.0)

if waypoints:
    print(f"Path created with {len(waypoints)} waypoints")
    
    # Execute each waypoint
    for i, (x, y, z) in enumerate(waypoints):
        print(f"Moving to waypoint {i}: ({x:.2f}, {y:.2f}, {z:.2f})")
        # Send to servos here
```

### Planning Methods

#### 1. Smooth Path (Joint Space Interpolation)
Best for most applications - ensures smooth servo motion.

```python
waypoints = planner.plan_smooth_path(start_xyz, end_xyz, max_joint_change=5.0)
```

Parameters:
- `start_xyz`: Starting position (x, y, z) in cm
- `end_xyz`: Target position (x, y, z) in cm
- `max_joint_change`: Maximum degrees change per waypoint (default: 5.0°)

#### 2. Linear Path (Cartesian Space)
Creates a straight line in XYZ space - may cause jerky joint motion.

```python
waypoints = planner.plan_linear_path(start_xyz, end_xyz, num_waypoints=20)
```

Parameters:
- `start_xyz`: Starting position (x, y, z) in cm
- `end_xyz`: Target position (x, y, z) in cm
- `num_waypoints`: Number of intermediate waypoints (default: 20)

#### 3. Best Path (Recommended)
Automatically selects the best planning method.

```python
waypoints = planner.plan_best_path(start_xyz, end_xyz, max_joint_change=5.0)
```

### Example Output

```bash
$ python3 path_planner.py --start 0 -31 29 --end -32 12 26 --max-change 5

============================================================
PATH PLANNER TEST
============================================================

============================================================
Test: Move with 5.0° max joint changes

🔍 Planning smooth path...
   From: (0.00, -31.00, 29.00) cm
   To:   (-32.00, 12.00, 26.00) cm
   Max joint change: 5.0° per step
   Max angle change needed: 47.23°
   Generating 10 waypoints...
   Waypoint 0/9: (0.00, -31.00, 29.00) cm
   Waypoint 2/9: (-12.45, -21.34, 28.12) cm
   Waypoint 4/9: (-22.18, -9.82, 27.28) cm
   Waypoint 6/9: (-28.56, 3.21, 26.62) cm
   Waypoint 9/9: (-32.00, 12.00, 26.00) cm
✓ Path created with 10 waypoints

✓ Path with 10 waypoints:

   Waypoint breakdown:
      0: (   0.00,  -31.00,   29.00) cm
      1: (  -6.89,  -26.42,   28.56) cm
      2: ( -12.45,  -21.34,   28.12) cm
      ...

   Total path length: 46.82 cm
   Average step size: 5.20 cm

============================================================
```

### Integration with ROS2/Servos

```python
# After planning, execute the path
for waypoint in waypoints:
    x, y, z = waypoint
    
    # Get joint angles for this waypoint
    angles = kinematics.inverse_kinematics(x, y, z)
    
    if angles:
        # Send angles to your servo controller
        # servo_controller.move_to_angles(angles)
        
        # Wait for movement to complete
        # time.sleep(0.1)
        pass
```

### Tips

- **Smoother Motion**: Use smaller `max_joint_change` values (3-5°)
- **Faster Motion**: Use larger `max_joint_change` values (10-15°)
- **Obstacle Avoidance**: Check each waypoint before execution
- **Path Validation**: Always check if path is `None` before executing
- **Test First**: Use command-line testing to verify paths before using in code

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
| `teleop_keyboard.py` | Keyboard teleoperation with XYZ display |
| `path_executor.py` | **Execute planned paths via ROS2** |
| `path_planner.py` | Path planning algorithm (testing only) |
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
