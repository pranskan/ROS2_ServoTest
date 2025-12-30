"""
Forward Kinematics Calibration Tool
-----------------------------------
Measure actual servo mounting angles to fix FK calculations.

This will help you find the offset angles so FK displays correct XYZ positions.
"""

import math


def analyze_teleop_data():
    """
    Analyze your teleop data to find servo mounting offsets.
    
    From your data:
    - All servos at 90° → XYZ: (23.56, -10.99, 23.40) cm
    
    If servos were truly horizontal at 90°:
    - Expected: arm pointing straight forward horizontally
    - Height would be: L1 = 10.5 cm
    - Forward reach: L2 + L3 + L4 = 38.9 cm
    
    But actual position shows:
    - Height: 23.4 cm (12.9 cm ABOVE base!)
    - Forward: only 26 cm (not 38.9)
    
    This means the arm is angled UPWARD at 90°
    """
    
    print("=" * 70)
    print("FORWARD KINEMATICS CALIBRATION")
    print("=" * 70)
    
    # Your measurements
    L1 = 10.5  # Base height
    L2 = 12.9  # Upper arm
    L3 = 11.0  # Forearm
    L4 = 15.0  # Gripper
    
    # Actual position when all at 90° (from your teleop data)
    actual_x = 23.56
    actual_y = -10.99
    actual_z = 23.40
    actual_r = math.sqrt(actual_x**2 + actual_y**2)  # Horizontal distance
    
    print(f"\nRobot link lengths:")
    print(f"  L1 (base height):  {L1} cm")
    print(f"  L2 (upper arm):    {L2} cm")
    print(f"  L3 (forearm):      {L3} cm")
    print(f"  L4 (gripper):      {L4} cm")
    print(f"  Total arm length:  {L2 + L3 + L4} cm")
    
    print(f"\nActual position when all servos at 90°:")
    print(f"  X: {actual_x:.2f} cm")
    print(f"  Y: {actual_y:.2f} cm")
    print(f"  Z: {actual_z:.2f} cm")
    print(f"  Horizontal distance: {actual_r:.2f} cm")
    
    # Calculate what the angle actually is
    height_above_base = actual_z - L1
    print(f"\nHeight above base (shoulder): {height_above_base:.2f} cm")
    
    # If arm is straight, we can calculate the actual angle
    # tan(angle) = height / horizontal
    actual_angle_rad = math.atan2(height_above_base, actual_r)
    actual_angle_deg = math.degrees(actual_angle_rad)
    
    print(f"\nCalculated arm angle from horizontal: {actual_angle_deg:.1f}°")
    print(f"This means when shoulder servo = 90°, mechanical angle ≈ {actual_angle_deg:.1f}°")
    
    # Calculate the offset
    servo_offset = actual_angle_deg
    
    print("\n" + "=" * 70)
    print("CALIBRATION RESULTS")
    print("=" * 70)
    print(f"\nServo mounting offset: ~{servo_offset:.1f}°")
    print(f"\nThis means:")
    print(f"  - Servo at 90° → Mechanical angle ~{servo_offset:.1f}° from horizontal")
    print(f"  - Servo at 0° → Mechanical angle ~{-90 + servo_offset:.1f}°")
    print(f"  - Servo at 180° → Mechanical angle ~{90 + servo_offset:.1f}°")
    
    print("\n" + "=" * 70)
    print("NEXT STEPS:")
    print("=" * 70)
    print("\n1. Update forward_kinematics() in kinematics.py:")
    print(f"   Add offset: theta1 += math.radians({servo_offset:.1f})")
    print("\n2. Test with teleop to verify XYZ positions are now correct")
    print("\n3. Once FK is correct, we can fix IK to match")
    print("=" * 70)
    
    return servo_offset


if __name__ == '__main__':
    offset = analyze_teleop_data()
    
    print("\n\nWould you like me to show you the exact code changes?")
    print("The fix is simple - just add the offset to the FK calculations.")
