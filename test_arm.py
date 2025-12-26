"""
Interactive test script for 6-servo robotic arm
Control each servo individually by entering motor number and angle
"""
import time
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio
from kinematics import ArmKinematics

print("=" * 60)
print("INTERACTIVE 6-SERVO ROBOTIC ARM CONTROL")
print("=" * 60)
print()

# Initialize PCA9685
print("Initializing PCA9685...")
try:
    i2c = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c)
    pca.frequency = 60
    print("✓ PCA9685 initialized at 60Hz")
except Exception as e:
    print(f"✗ Failed to initialize PCA9685: {e}")
    exit(1)

print()

# Initialize kinematics solver
arm_kinematics = ArmKinematics()

# Servo configuration
SERVO_MIN = 0x0CCC  # ~0.5ms pulse
SERVO_MAX = 0x1999  # ~2.5ms pulse
NUM_SERVOS = 6

servo_names = [
    "Base",
    "Shoulder",
    "Elbow",
    "Wrist Pitch",
    "Wrist Roll",
    "Gripper"
]

# Track current positions
current_positions = [90] * NUM_SERVOS

def set_angle(channel, angle):
    """Set servo angle."""
    angle = max(0, min(180, angle))
    pulse = int(SERVO_MIN + (angle / 180.0) * (SERVO_MAX - SERVO_MIN))
    pca.channels[channel].duty_cycle = pulse
    current_positions[channel] = angle
    return angle

def reset_all_servos():
    """Reset all servos - disable all PWM outputs to stop current draw."""
    print("\nResetting all servos (disabling PWM outputs)...")
    for channel in range(NUM_SERVOS):
        # Set duty cycle to 0 to completely disable PWM output
        pca.channels[channel].duty_cycle = 0
        current_positions[channel] = 0
    print("✓ All PWM outputs disabled")

def show_menu():
    """Display the control menu."""
    print()
    print("=" * 60)
    print("SERVO CONTROL MENU")
    print("=" * 60)
    print()
    print("Motors (Current Positions):")
    print("-" * 60)
    for i in range(NUM_SERVOS):
        print(f"  {i}: {servo_names[i]:15s} - {current_positions[i]:5.1f}°")
    print()
    print("Commands:")
    print("-" * 60)
    print("  <motor> <angle>  - Move motor to angle (e.g., '0 90')")
    print("  all <angle>      - Move all motors to angle (e.g., 'all 90')")
    print("  xyz <x> <y> <z>  - Move to XYZ position in cm (e.g., 'xyz 20 0 30')")
    print("  center           - Move all motors to 90°")
    print("  home             - Move all motors to home position (90°)")
    print("  reset / stop     - Disable all PWM outputs (stop current draw)")
    print("  status           - Show current positions")
    print("  help             - Show this menu")
    print("  quit / exit      - Exit program")
    print("=" * 60)
    print()

def parse_command(cmd):
    """Parse user command and execute it."""
    parts = cmd.lower().strip().split()
    
    if not parts:
        return True
    
    command = parts[0]
    
    # Exit commands
    if command in ['quit', 'exit', 'q']:
        return False
    
    # Help command
    if command in ['help', 'h', '?']:
        show_menu()
        return True
    
    # Status command
    if command in ['status', 's']:
        print("\nCurrent Positions:")
        for i in range(NUM_SERVOS):
            print(f"  Motor {i} ({servo_names[i]}): {current_positions[i]:.1f}°")
        print()
        return True
    
    # Reset command - disable all PWM
    if command in ['reset', 'stop', 'off']:
        reset_all_servos()
        return True
    
    # Center/Home commands
    if command in ['center', 'home']:
        print("\nMoving all motors to 90° (center)...")
        for i in range(NUM_SERVOS):
            set_angle(i, 90)
            print(f"  Motor {i} ({servo_names[i]}): 90°")
        print("✓ Done!")
        return True
    
    # All motors command
    if command == 'all':
        if len(parts) < 2:
            print("✗ Usage: all <angle>")
            print("  Example: all 90")
            return True
        
        try:
            angle = float(parts[1])
            if angle < 0 or angle > 180:
                print("✗ Angle must be between 0 and 180")
                return True
            
            print(f"\nMoving all motors to {angle}°...")
            for i in range(NUM_SERVOS):
                set_angle(i, angle)
                print(f"  Motor {i} ({servo_names[i]}): {angle}°")
            print("✓ Done!")
            
        except ValueError:
            print("✗ Invalid angle. Must be a number between 0 and 180")
        
        return True
    
    # XYZ positioning command
    if command == 'xyz':
        if len(parts) < 4:
            print("✗ Usage: xyz <x> <y> <z>")
            print("  Example: xyz 20 0 30")
            print("  Coordinates in cm")
            return True
        
        try:
            x = float(parts[1])
            y = float(parts[2])
            z = float(parts[3])
            
            print(f"\nCalculating angles for position ({x}, {y}, {z}) cm...")
            angles = arm_kinematics.inverse_kinematics(x, y, z)
            
            if angles:
                print(f"\n✓ Solution found! Moving to position...")
                for i, angle in enumerate(angles):
                    set_angle(i, angle)
                    print(f"  Motor {i} ({servo_names[i]}): {angle:.1f}°")
                print("✓ Done!")
            else:
                print("✗ Position unreachable with current arm configuration")
            
        except ValueError:
            print("✗ Invalid coordinates. Must be numbers.")
        
        return True
    
    # Individual motor command
    try:
        motor = int(parts[0])
        
        if motor < 0 or motor >= NUM_SERVOS:
            print(f"✗ Motor must be between 0 and {NUM_SERVOS-1}")
            return True
        
        if len(parts) < 2:
            print("✗ Usage: <motor> <angle>")
            print("  Example: 0 90")
            return True
        
        angle = float(parts[1])
        
        if angle < 0 or angle > 180:
            print("✗ Angle must be between 0 and 180")
            return True
        
        actual_angle = set_angle(motor, angle)
        print(f"✓ Motor {motor} ({servo_names[motor]}) moved to {actual_angle:.1f}°")
        
    except ValueError:
        print("✗ Invalid command. Type 'help' for available commands")
    
    return True

# Initialize all servos to center
print("Initializing all servos to 90° (center)...")
for i in range(NUM_SERVOS):
    set_angle(i, 90)
    print(f"  Motor {i} ({servo_names[i]}): 90°")
print()
print("✓ Initialization complete!")
print()
print("Kinematics Configuration:")
print(f"  Max reach: {arm_kinematics.L2 + arm_kinematics.L3 + arm_kinematics.L4:.2f} cm")
print(f"  Link lengths: L1={arm_kinematics.L1}, L2={arm_kinematics.L2}, L3={arm_kinematics.L3}, L4={arm_kinematics.L4} cm")

# Show initial menu
show_menu()

# Main control loop
try:
    while True:
        try:
            command = input("Enter command: ")
            if not parse_command(command):
                break
        except KeyboardInterrupt:
            print("\n\nExiting...")
            break
        except Exception as e:
            print(f"✗ Error: {e}")

except KeyboardInterrupt:
    print("\n\nExiting...")

# Disable all PWM outputs before exit to stop current draw
print("\nDisabling all PWM outputs...")
reset_all_servos()
print("✓ Done!")
print("\nGoodbye!")
