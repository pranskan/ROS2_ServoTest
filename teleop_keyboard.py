"""
Keyboard Teleoperation for Robotic Arm
--------------------------------------
Control arm with keyboard and display real-time XYZ position.
Press keys to move motors, Ctrl+C to exit.
"""
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import threading
from kinematics_dh import ArmKinematicsDH

# Motor names
MOTOR_NAMES = [
    "Gripper",      # 0
    "Wrist Roll",   # 1
    "Wrist Pitch",  # 2
    "Elbow",        # 3
    "Shoulder",     # 4
    "Base"          # 5
]

# Servo angle limits (min, max)
SERVO_LIMITS = [
    (0, 180),       # 0: Gripper - 180° servo
    (0, 180),       # 1: Wrist Roll - 180° servo
    (0, 270),       # 2: Wrist Pitch - 270° servo
    (0, 270),       # 3: Elbow - 270° servo
    (0, 270),       # 4: Shoulder - 270° servo
    (0, 270),       # 5: Base - 270° servo
]

# Key mappings
KEY_HELP = """
====================================================
KEYBOARD TELEOPERATION CONTROLS
====================================================

Motor Selection (press number):
  0 - Gripper (0-180°)
  1 - Wrist Roll (0-180°)
  2 - Wrist Pitch (0-270°)
  3 - Elbow (0-270°)
  4 - Shoulder (0-270°)
  5 - Base (0-270°)

Movement (after selecting motor):
  + / = : Increase angle by 5°
  - / _ : Decrease angle by 5°
  ] : Increase angle by 1°
  [ : Decrease angle by 1°

Quick Commands:
  c : Center all
  s : Show status
  h : Show help

Movement Speed:
  f : Fast mode (10° increments)
  n : Normal mode (5° increments)
  m : Slow mode (1° increments)

Exit:
  q / Ctrl+C : Quit

====================================================
"""


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard_node')
        
        # Publisher
        self.arm_pub = self.create_publisher(Float32MultiArray, 'arm_command', 10)
        
        # Initialize kinematics
        self.arm_kinematics = ArmKinematicsDH(
            base_height=10.5,
            l2=12.9,
            l3=11.0,
            l4=15.0
        )
        
        # Current motor angles - start at center positions
        self.angles = [90.0, 90.0, 135.0, 135.0, 135.0, 135.0]
        
        # Current selected motor
        self.selected_motor = 5  # Default to base
        
        # Movement increment
        self.increment = 5.0  # Default increment
        
        # Timer to publish current state
        self.timer = self.create_timer(0.5, self.publish_state)
        
        self.get_logger().info('Keyboard Teleoperation Node Started')
        self.get_logger().info(f'Selected motor: {MOTOR_NAMES[self.selected_motor]}')
    
    def publish_state(self):
        """Publish current angles."""
        msg = Float32MultiArray()
        msg.data = self.angles
        self.arm_pub.publish(msg)
    
    def get_xyz_position(self):
        """Calculate current XYZ position from motor angles."""
        pos = self.arm_kinematics.forward_kinematics(self.angles)
        return pos
    
    def move_motor(self, motor, delta):
        """Move a motor by delta degrees."""
        min_limit, max_limit = SERVO_LIMITS[motor]
        self.angles[motor] = max(min_limit, min(max_limit, self.angles[motor] + delta))
        self.publish_state()
        
        # Get and display XYZ position
        xyz = self.get_xyz_position()
        print(f"{MOTOR_NAMES[motor]}: {self.angles[motor]:.1f}° | XYZ: ({xyz['x']:.2f}, {xyz['y']:.2f}, {xyz['z']:.2f}) cm")
    
    def center_all(self):
        """Move all motors to center position."""
        self.angles = [90.0, 90.0, 135.0, 135.0, 135.0, 135.0]
        self.publish_state()
        
        # Display XYZ position
        xyz = self.get_xyz_position()
        self.get_logger().info('Centered all motors')
        print(f"Position: ({xyz['x']:.2f}, {xyz['y']:.2f}, {xyz['z']:.2f}) cm")
    
    def show_status(self):
        """Print current status with XYZ position."""
        xyz = self.get_xyz_position()
        
        print("\n" + "=" * 60)
        print("CURRENT STATUS")
        print("=" * 60)
        print(f"\nEnd-Effector Position (XYZ):")
        print(f"  X: {xyz['x']:7.2f} cm")
        print(f"  Y: {xyz['y']:7.2f} cm")
        print(f"  Z: {xyz['z']:7.2f} cm")
        print(f"\nJoint Angles:")
        for i, (name, angle) in enumerate(zip(MOTOR_NAMES, self.angles)):
            min_lim, max_lim = SERVO_LIMITS[i]
            marker = " <--" if i == self.selected_motor else ""
            print(f"  {i}: {name:15s} = {angle:6.1f}° ({min_lim}-{max_lim}°){marker}")
        print(f"\nIncrement: {self.increment}°")
        print("=" * 60 + "\n")


def get_key():
    """Get a single keypress from terminal."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def main():
    print(KEY_HELP)
    
    rclpy.init()
    node = TeleopNode()
    
    # Run ROS2 spin in separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    # Display initial position
    xyz = node.get_xyz_position()
    print("Ready! Select a motor (0-5) and use +/- to move it.")
    print(f"Currently controlling: {MOTOR_NAMES[node.selected_motor]}")
    print(f"Current position: ({xyz['x']:.2f}, {xyz['y']:.2f}, {xyz['z']:.2f}) cm")
    print("Press 'h' for help, 'q' to quit.\n")
    
    try:
        while True:
            key = get_key()
            
            # Quit
            if key == 'q' or key == '\x03':
                break
            
            # Select motor
            elif key in '012345':
                node.selected_motor = int(key)
                xyz = node.get_xyz_position()
                min_lim, max_lim = SERVO_LIMITS[node.selected_motor]
                print(f"Selected: {MOTOR_NAMES[node.selected_motor]} ({min_lim}-{max_lim}°) | Position: ({xyz['x']:.2f}, {xyz['y']:.2f}, {xyz['z']:.2f}) cm")
            
            # Increase angle
            elif key in ['+', '=', ']']:
                delta = 1.0 if key == ']' else node.increment
                node.move_motor(node.selected_motor, delta)
            
            # Decrease angle
            elif key in ['-', '_', '[']:
                delta = 1.0 if key == '[' else node.increment
                node.move_motor(node.selected_motor, -delta)
            
            # Center all
            elif key == 'c':
                node.center_all()
            
            # Show status
            elif key == 's':
                node.show_status()
            
            # Speed modes
            elif key == 'f':
                node.increment = 10.0
                print(f"Fast mode: {node.increment}° increments")
            elif key == 'n':
                node.increment = 5.0
                print(f"Normal mode: {node.increment}° increments")
            elif key == 'm':
                node.increment = 1.0
                print(f"Slow mode: {node.increment}° increments")
            
            # Help
            elif key == 'h':
                print(KEY_HELP)
    
    except KeyboardInterrupt:
        pass
    
    finally:
        print("\nShutting down...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
