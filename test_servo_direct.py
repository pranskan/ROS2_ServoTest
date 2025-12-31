"""
Direct servo test - no ROS2
Test if PCA9685 and servos are working
"""

from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio
import time

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50

print("PCA9685 Direct Test")
print("=" * 60)
print(f"Frequency: {pca.frequency} Hz")
print(f"Testing channel 5 (Base motor)")
print("=" * 60)

# 180° servo: 3276 to 6553
MIN_180 = 3276
MAX_180 = 6553

# 270° servo: 1638 to 8191
MIN_270 = 1638
MAX_270 = 8191

try:
    # Start with min position
    print("\nSetting to minimum position (500µs)...")
    pca.channels[5].duty_cycle = MIN_270
    time.sleep(2)
    print("Should have moved to one extreme. Press Enter...")
    input()
    
    # Center
    print("Setting to center (1500µs)...")
    center = int((MIN_270 + MAX_270) / 2)
    pca.channels[5].duty_cycle = center
    time.sleep(2)
    print("Should be centered. Press Enter...")
    input()
    
    # Max position
    print("Setting to maximum position (2500µs)...")
    pca.channels[5].duty_cycle = MAX_270
    time.sleep(2)
    print("Should have moved to other extreme. Press Enter...")
    input()
    
    # Off
    print("Disabling...")
    pca.channels[5].duty_cycle = 0
    
    print("\nTest complete!")
    
except Exception as e:
    print(f"Error: {e}")
