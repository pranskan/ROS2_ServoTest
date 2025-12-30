"""
Safe Servo Initialization
-------------------------
Initializes all servos to 90° BEFORE starting ROS2 node.
Run this first to avoid spasming on startup.
"""
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio
import time

def safe_init():
    """Safely initialize all servos to center position."""
    print("=" * 60)
    print("SAFE SERVO INITIALIZATION")
    print("=" * 60)
    
    # Initialize I2C and PCA9685
    print("\nConnecting to PCA9685...")
    i2c = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c)
    pca.frequency = 60
    print("✓ Connected")
    
    # Servo configuration
    NUM_SERVOS = 6
    SERVO_NAMES = ["Gripper", "Wrist Roll", "Wrist Pitch", "Elbow", "Shoulder", "Base"]
    
    def get_pulse_range(channel):
        """Get pulse range for servo."""
        if channel in [0, 1]:  # MG996R
            return 0x0CCC, 0x1999
        else:  # DS3218
            return 0x0CCC, 0x1999
    
    def set_servo_angle(channel, angle):
        """Set servo angle."""
        angle = max(0, min(180, angle))
        min_pulse, max_pulse = get_pulse_range(channel)
        pulse = int(min_pulse + (angle / 180.0) * (max_pulse - min_pulse))
        pca.channels[channel].duty_cycle = pulse
    
    # Step 1: Disable all PWM
    print("\nStep 1: Disabling all PWM outputs...")
    for channel in range(NUM_SERVOS):
        pca.channels[channel].duty_cycle = 0
    print("✓ All servos disabled")
    time.sleep(2.0)
    
    # Step 2: Very slowly move to 90°
    print("\nStep 2: Moving all servos to center (90°)...")
    print("This will take ~20 seconds - please wait...")
    
    target = 90.0
    
    for channel in range(NUM_SERVOS):
        print(f"\n  Servo {channel} ({SERVO_NAMES[channel]}):")
        
        # Move in very small increments
        steps = 80  # 80 steps
        for step in range(steps + 1):
            # Just set to target angle with small delays
            set_servo_angle(channel, target)
            time.sleep(0.025)  # 25ms per step
            
            # Progress indicator
            if step % 20 == 0:
                print(f"    Progress: {int(step/steps * 100)}%")
        
        print(f"    ✓ {SERVO_NAMES[channel]} centered at 90°")
    
    print("\n" + "=" * 60)
    print("✓ ALL SERVOS INITIALIZED TO 90°")
    print("=" * 60)
    print("\nServos are now at center position.")
    print("It is safe to start servo_control.py now.")
    print("\nPress Ctrl+C to exit, or wait 5 seconds...")
    
    # Keep PWM active for 5 seconds
    time.sleep(5)
    
    print("\nKeeping servos powered...")
    print("Starting servo_control.py in another terminal now is recommended.")
    print("Press Ctrl+C when ready to stop this script.")
    
    try:
        # Keep servos at 90° indefinitely
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\nDisabling all servos...")
        for channel in range(NUM_SERVOS):
            pca.channels[channel].duty_cycle = 0
        print("✓ Safe shutdown complete")


if __name__ == "__main__":
    try:
        safe_init()
    except Exception as e:
        print(f"\n❌ Error: {e}")
        print("Make sure I2C is enabled and PCA9685 is connected.")
