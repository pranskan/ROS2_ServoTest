"""
Measure arm dimensions for DH parameter calculation
"""

print("""
====================================================
ARM DIMENSION MEASUREMENT FOR DH PARAMETERS
====================================================

You need to measure your arm and identify:
1. Base height (Z when all joints at 90°)
2. Upper arm length (shoulder pivot to elbow pivot)
3. Forearm length (elbow pivot to wrist pivot)
4. Gripper length (wrist pivot to end effector)

MEASUREMENT INSTRUCTIONS:
========================

1. Place arm flat on table, all servos at 90°
2. Measure from table to base attachment point → BASE_HEIGHT

3. Measure shoulder servo center to elbow servo center → L2_LENGTH
   (This is the distance the upper arm spans)

4. Measure elbow servo center to wrist servo center → L3_LENGTH
   (This is the forearm length)

5. Measure wrist servo center to gripper tip → L4_LENGTH
   (Distance from wrist to end effector)

====================================================
MEASUREMENT FORM:
====================================================
""")

# Get measurements from user
base_height = float(input("Base height (cm): "))
l2_length = float(input("Upper arm length - Shoulder to Elbow (cm): "))
l3_length = float(input("Forearm length - Elbow to Wrist (cm): "))
l4_length = float(input("Gripper length - Wrist to End (cm): "))

print("\n" + "=" * 60)
print("MEASUREMENTS RECORDED")
print("=" * 60)
print(f"Base Height:    {base_height:.1f} cm")
print(f"Upper Arm (L2): {l2_length:.1f} cm")
print(f"Forearm (L3):   {l3_length:.1f} cm")
print(f"Gripper (L4):   {l4_length:.1f} cm")
print("=" * 60)

# Calculate DH parameters
print("\n" + "=" * 60)
print("DH PARAMETERS")
print("=" * 60)
print("""
For a 6-DOF arm with configuration:
  Joint 1: Base rotation (around Z axis)
  Joint 2: Shoulder pitch (around Y axis)
  Joint 3: Elbow pitch (around Y axis)
  Joint 4: Wrist pitch (around Y axis)
  Joint 5: Wrist roll (around Z axis)
  Joint 6: Gripper (not part of FK)

DH Table:
""")

dh_params = [
    {"joint": "Base (rotation)", "a": 0, "alpha": 0, "d": base_height, "theta": "θ1"},
    {"joint": "Shoulder", "a": 0, "alpha": 90, "d": 0, "theta": "θ2"},
    {"joint": "Upper Arm", "a": l2_length, "alpha": 0, "d": 0, "theta": "θ3"},
    {"joint": "Elbow", "a": 0, "alpha": 90, "d": 0, "theta": "θ4"},
    {"joint": "Forearm", "a": l3_length, "alpha": 0, "d": 0, "theta": "θ5"},
    {"joint": "Wrist", "a": 0, "alpha": 90, "d": 0, "theta": "θ6"},
    {"joint": "Gripper", "a": l4_length, "alpha": 0, "d": 0, "theta": "0"},
]

for i, param in enumerate(dh_params, 1):
    print(f"\nLink {i}: {param['joint']}")
    print(f"  a     = {param['a']:.1f} cm")
    print(f"  alpha = {param['alpha']}°")
    print(f"  d     = {param['d']:.1f} cm")
    print(f"  theta = {param['theta']}")

print("\n" + "=" * 60)
print("Save these values for kinematics.py!")
print("=" * 60)
