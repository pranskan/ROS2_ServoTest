"""
Servo Calibration Script
------------------------
Find the correct MIN and MAX pulse values for your specific servos.
"""
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio
import time

print("=" * 60)
print("SERVO CALIBRATION TOOL")
print("=" * 60)
print()
print("This script helps you find the correct pulse values")
print("for full 180° servo movement.")
print()

# Initialize PCA9685
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 60

# Test on channel 0 (change if needed)
CHANNEL = 0

print(f"Testing on channel {CHANNEL}")
print()
print("INSTRUCTIONS:")
print("1. Watch the servo carefully")
print("2. Find the pulse value where servo is at 0° (far left)")
print("3. Find the pulse value where servo is at 180° (far right)")
print()

input("Press ENTER to start calibration...")
print()

# Start with a safe middle position
print("Moving to middle position (pulse = 375)...")
pca.channels[CHANNEL].duty_cycle = 375
time.sleep(2)

print()
print("=" * 60)
print("FINDING MINIMUM (0°)")
print("=" * 60)
print()

# Test minimum values
test_min_values = [100, 150, 200, 250, 300]
print("Testing different MIN pulse values...")
print("Watch which position is closest to 0° (far left)")
print()

for pulse in test_min_values:
    print(f"Testing pulse = {pulse}")
    pca.channels[CHANNEL].duty_cycle = pulse
    time.sleep(2)

print()
min_pulse = int(input("Enter the pulse value that looked best for 0°: "))
print(f"✓ MIN pulse set to: {min_pulse}")
print()

# Return to center
print("Returning to center...")
pca.channels[CHANNEL].duty_cycle = 375
time.sleep(2)

print()
print("=" * 60)
print("FINDING MAXIMUM (180°)")
print("=" * 60)
print()

# Test maximum values
test_max_values = [450, 500, 550, 600, 650]
print("Testing different MAX pulse values...")
print("Watch which position is closest to 180° (far right)")
print()

for pulse in test_max_values:
    print(f"Testing pulse = {pulse}")
    pca.channels[CHANNEL].duty_cycle = pulse
    time.sleep(2)

print()
max_pulse = int(input("Enter the pulse value that looked best for 180°: "))
print(f"✓ MAX pulse set to: {max_pulse}")
print()

# Final test
print("=" * 60)
print("FINAL TEST")
print("=" * 60)
print()
print(f"Testing full range with MIN={min_pulse}, MAX={max_pulse}")
print()

positions = [
    (min_pulse, "0° (MIN)"),
    ((min_pulse + max_pulse) // 2, "90° (CENTER)"),
    (max_pulse, "180° (MAX)"),
    ((min_pulse + max_pulse) // 2, "90° (CENTER)"),
]

for pulse, description in positions:
    print(f"{description}: pulse = {pulse}")
    pca.channels[CHANNEL].duty_cycle = pulse
    time.sleep(2)

print()
print("=" * 60)
print("CALIBRATION COMPLETE!")
print("=" * 60)
print()
print(f"Your calibrated values:")
print(f"  SERVO_MIN = {min_pulse}")
print(f"  SERVO_MAX = {max_pulse}")
print()
print("Update these values in:")
print("  - test_arm.py")
print("  - servo_control.py")
print()

# Disable PWM
pca.channels[CHANNEL].duty_cycle = 0
print("✓ Servo disabled")
