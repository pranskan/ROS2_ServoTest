"""
Test PWM pulse values for servos
Correct values based on actual servo specs
"""

# PCA9685 timing: 20ms period / 4096 steps = 4.88µs per step

# 270° servo: 500µs to 2500µs
MIN_270 = 0x0066  # 500µs ÷ 4.88µs = 102
MAX_270 = 0x0200  # 2500µs ÷ 4.88µs = 512

# 180° servo: 1000µs to 2000µs
MIN_180 = 0x00CD  # 1000µs ÷ 4.88µs = 205
MAX_180 = 0x019A  # 2000µs ÷ 4.88µs = 410

def usec_to_pwm(microseconds):
    """Convert microseconds to PWM value."""
    return int(microseconds / 4.88)

def pwm_to_usec(pwm_value):
    """Convert PWM value to microseconds."""
    return pwm_value * 4.88

def test_pulse_mapping(servo_type, angles_to_test):
    """Test pulse values for given angles."""
    if servo_type == 270:
        min_pulse, max_pulse = MIN_270, MAX_270
        max_angle = 270
        min_usec, max_usec = 500, 2500
    else:
        min_pulse, max_pulse = MIN_180, MAX_180
        max_angle = 180
        min_usec, max_usec = 1000, 2000
    
    print(f"\n{servo_type}° Servo Pulse Mapping")
    print(f"Range: {min_usec}µs to {max_usec}µs")
    print(f"PWM:   0x{min_pulse:04X} ({min_pulse}) to 0x{max_pulse:04X} ({max_pulse})")
    print(f"\nAngle → PWM (dec) | PWM (hex) | µs    | PWM% ")
    print("-" * 55)
    
    for angle in angles_to_test:
        pulse = int(min_pulse + (angle / max_angle) * (max_pulse - min_pulse))
        usec = pwm_to_usec(pulse)
        pwm_percent = (pulse / 4096) * 100
        print(f"{angle:3.0f}° → {pulse:5d} | 0x{pulse:04X} | {usec:6.0f} | {pwm_percent:5.1f}%")

print("=" * 55)
print("PCA9685 PWM CALCULATION")
print("=" * 55)
print(f"Frequency: 50Hz (20ms period)")
print(f"Resolution: 12-bit (4096 steps)")
print(f"Step size: 20ms / 4096 = 4.88µs\n")

# Test 270° servo
test_pulse_mapping(270, [0, 45, 90, 135, 180, 225, 270])

# Test 180° servo  
test_pulse_mapping(180, [0, 45, 90, 135, 180])

print("\n" + "=" * 55)
print("SERVO SPECIFICATIONS")
print("=" * 55)
print("180° servo (MG996R):")
print("  Range: 1000µs to 2000µs")
print("  PWM: 0x00CD to 0x019A")
print("\n270° servo (DS3218):")
print("  Range: 500µs to 2500µs")
print("  PWM: 0x0066 to 0x0200")
print("=" * 55)
