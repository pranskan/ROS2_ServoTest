"""
Configuration file for robotic arm settings.
Edit this file to change servo specs, limits, and movement parameters.
"""

# Servo specifications: (min_angle, max_angle, is_270_degree)
SERVO_SPECS = [
    (0, 180, False),      # 0: Gripper - 180° servo
    (0, 180, False),      # 1: Wrist Roll - 180° servo
    (0, 180, True),       # 2: Wrist Pitch - 270° servo limited to 180°
    (0, 180, True),       # 3: Elbow - 270° servo limited to 180°
    (0, 180, True),       # 4: Shoulder - 270° servo limited to 180°
    (0, 180, True),       # 5: Base - 270° servo limited to 180°
]

# Servo angle limits (min, max) - matches SERVO_SPECS
SERVO_LIMITS = [
    (0, 180),       # 0: Gripper - 180° servo
    (0, 180),       # 1: Wrist Roll - 180° servo
    (0, 180),       # 2: Wrist Pitch - 270° servo limited to 180°
    (0, 180),       # 3: Elbow - 270° servo limited to 180°
    (0, 180),       # 4: Shoulder - 270° servo limited to 180°
    (0, 180),       # 5: Base - 270° servo limited to 180°
]

# Pulse ranges: (min_pulse, max_pulse) for each servo type
PULSE_RANGES = [
    (1966, 8737),  # 180° servo: 1000-2000µs
    (1966, 8737),  # 270° servo: 500-1833µs (adjust if needed)
]

# Initial angles (set each motor separately for homing)
INITIAL_ANGLES = [
    0.0,  # 0: Gripper
    0.0,  # 1: Wrist Roll
    0.0,  # 2: Wrist Pitch
    0.0,  # 3: Elbow
    0.0,  # 4: Shoulder
    0.0,  # 5: Base
]

# Motor names
MOTOR_NAMES = [
    "Gripper",      # 0
    "Wrist Roll",   # 1
    "Wrist Pitch",  # 2
    "Elbow",        # 3
    "Shoulder",     # 4
    "Base"          # 5
]

# Movement parameters
MOVEMENT_SPEED = 2.0  # degrees per step (lower = smoother)
STEP_DELAY = 0.02     # seconds between steps (lower = faster)
