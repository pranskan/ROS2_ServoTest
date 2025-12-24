"""
Simple servo test without ROS2
Tests PCA9685 and servo connection directly
"""
import time
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

print("Initializing PCA9685...")
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 60
print("✓ PCA9685 initialized")

print("\nTesting servo on channel 0...")
print("Moving through positions...")

positions = [
    (0, "0° (left)"),
    (90, "90° (center)"),
    (180, "180° (right)"),
    (90, "90° (center)"),
]

for angle, description in positions:
    pulse = int(0x0CCC + (angle / 180.0) * (0x1999 - 0x0CCC))
    pca.channels[0].duty_cycle = pulse
    print(f"Position: {description}")
    time.sleep(2)

print("\n✓ Test complete!")
print("\nTo run with ROS2:")
print("  source /opt/ros/jazzy/setup.bash")
print("  source ~/ros2_servo_venv/bin/activate")
print("  python3 servo_control.py")
#oejfsoaj