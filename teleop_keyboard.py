"""
Keyboard Teleoperation for Robotic Arm
--------------------------------------
Control arm with keyboard and display real-time angle positions.
Press keys to move motors, Ctrl+C to exit.
"""
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import threading
from config import SERVO_LIMITS, MOTOR_NAMES, INITIAL_ANGLES, PULSE_RANGES, SERVO_SPECS

# Motor names
MOTOR_NAMES = MOTOR_NAMES  # Override if needed, but use config

# Servo angle limits (min, max)
SERVO_LIMITS = SERVO_LIMITS  # Use config

# Key mappings
KEY_HELP = """
====================================================
KEYBOARD TELEOPERATION CONTROLS
====================================================

Motor Selection (press number):
  0 - Gripper (0-180°)
  1 - Wrist Roll (0-180°)
  2 - Wrist Pitch (0-180°)
  3 - Elbow (0-180°)
  4 - Shoulder (0-180°)
  5 - Base (0-180°)

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
        
        # Current motor angles - start at 0° for homing
        self.angles = INITIAL_ANGLES.copy()
        
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
    
    def move_motor(self, motor, delta):
        """Move a motor by delta degrees."""
        min_limit, max_limit = SERVO_LIMITS[motor]
        self.angles[motor] = max(min_limit, min(max_limit, self.angles[motor] + delta))
        self.publish_state()
        
        # Calculate PWM pulse
        min_angle, max_angle, is_270 = SERVO_SPECS[motor]
        min_pulse, max_pulse = PULSE_RANGES[1 if is_270 else 0]
        pulse = int(min_pulse + (self.angles[motor] / max_angle) * (max_pulse - min_pulse))
        
        # Display angle and PWM
        print(f"{MOTOR_NAMES[motor]}: {self.angles[motor]:.1f}° | PWM: {pulse} ({pulse/65535*100:.1f}%)")
    
    def center_all(self):
        """Move all motors to center position (initial angles for homing)."""
        self.angles = INITIAL_ANGLES.copy()
        self.publish_state()
        
        self.get_logger().info('Centered all motors')
        print("All motors centered to initial positions")
    
    def show_status(self):
        """Print current status."""
        print("\n" + "=" * 60)
        print("CURRENT STATUS")
        print("=" * 60)
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
    print("Ready! Select a motor (0-5) and use +/- to move it.")
    print(f"Currently controlling: {MOTOR_NAMES[node.selected_motor]}")
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
                min_lim, max_lim = SERVO_LIMITS[node.selected_motor]
                print(f"Selected: {MOTOR_NAMES[node.selected_motor]} ({min_lim}-{max_lim}°)")
            
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
