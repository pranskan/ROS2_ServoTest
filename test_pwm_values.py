"""
Test PWM pulse values for servos
Correct values based on actual servo specs and 16-bit duty cycle
"""

# PCA9685 timing: 50Hz = 20ms period
# 16-bit duty cycle (0-65535)
# 1µs = 65535 / 20000 = 3.28 counts

MICROSEC_TO_COUNTS = 65535 / 20000  # 3.276

# 270° servo: 500µs to 2500µs
MIN_270 = int(500 * MICROSEC_TO_COUNTS)    # 1638
MAX_270 = int(2500 * MICROSEC_TO_COUNTS)   # 8191

# 180° servo: 1000µs to 2000µs
MIN_180 = int(1000 * MICROSEC_TO_COUNTS)   # 3276
MAX_180 = int(2000 * MICROSEC_TO_COUNTS)   # 6553

def usec_to_pwm(microseconds):
    """Convert microseconds to PWM value."""
    return int(microseconds * MICROSEC_TO_COUNTS)

def pwm_to_usec(pwm_value):
    """Convert PWM value to microseconds."""
    return pwm_value / MICROSEC_TO_COUNTS

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
    print(f"PWM:   {min_pulse} to {max_pulse}")
    print(f"\nAngle → PWM Count | µs    | PWM% ")
    print("-" * 50)
    
    for angle in angles_to_test:
        pulse = int(min_pulse + (angle / max_angle) * (max_pulse - min_pulse))
        usec = pwm_to_usec(pulse)
        pwm_percent = (pulse / 65535) * 100
        print(f"{angle:3.0f}° → {pulse:5d} | {usec:6.0f} | {pwm_percent:5.1f}%")

print("=" * 60)
print("PCA9685 PWM CALCULATION (16-bit duty cycle)")
print("=" * 60)
print(f"Frequency: 50Hz (20ms period)")
print(f"Resolution: 16-bit (0-65535 counts)")
print(f"Conversion: 1µs = {MICROSEC_TO_COUNTS:.2f} counts\n")

# Test 270° servo
test_pulse_mapping(270, [0, 45, 90, 135, 180, 225, 270])

# Test 180° servo  
test_pulse_mapping(180, [0, 45, 90, 135, 180])

print("\n" + "=" * 60)
print("SERVO SPECIFICATIONS (16-bit counts)")
print("=" * 60)
print(f"180° servo (MG996R):")
print(f"  Range: 1000µs to 2000µs")
print(f"  PWM: {MIN_180} to {MAX_180}")
print(f"\n270° servo (DS3218):")
print(f"  Range: 500µs to 2500µs")
print(f"  PWM: {MIN_270} to {MAX_270}")
print("=" * 60)
