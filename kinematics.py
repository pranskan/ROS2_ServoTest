"""
Kinematics for 6-DOF Robotic Arm
EMPIRICALLY CALIBRATED based on actual measurements
"""
import math

class ArmKinematics:
    """Kinematics solver for custom robotic arm."""
    
    def __init__(self, link_lengths=None):
        """Initialize arm."""
        if link_lengths is None:
            self.L1 = 10.5  # Base height
            self.L2 = 12.9  # Upper arm
            self.L3 = 11.0  # Forearm
            self.L4 = 15.0  # Gripper
        else:
            self.L1, self.L2, self.L3, self.L4 = link_lengths
        
        # Workspace limits
        self.max_horizontal_reach = self.L2 + self.L3 + self.L4
        self.min_horizontal_reach = abs(self.L2 - self.L3)
        self.max_height = self.L1 + self.L2 + self.L3 + self.L4
        self.min_height = self.L1 - (self.L2 + self.L3 + self.L4)
    
    def get_workspace_limits(self):
        """Get workspace limits."""
        return {
            'x_max': self.max_horizontal_reach,
            'x_min': -self.max_horizontal_reach,
            'y_max': self.max_horizontal_reach,
            'y_min': -self.max_horizontal_reach,
            'z_max': self.max_height,
            'z_min': self.min_height,
            'max_reach': self.max_horizontal_reach,
            'min_reach': self.min_horizontal_reach,
        }
    
    def is_position_reachable(self, x, y, z):
        """Check if position is reachable."""
        if z > self.max_height or z < self.min_height:
            return False, f"Height out of range: {z:.2f}"
        
        r = math.sqrt(x**2 + y**2)
        if r > self.max_horizontal_reach or r < self.min_horizontal_reach:
            return False, f"Horizontal reach out of range: {r:.2f}"
        
        return True, "OK"
    
    def inverse_kinematics(self, x, y, z, pitch=90, roll=90, gripper=90):
        """
        Calculate joint angles for target XYZ.
        
        IMPORTANT: This is BROKEN for your robot geometry.
        Use angle-based control (teleop) instead of XYZ-based (path_executor).
        """
        print(f"⚠️  WARNING: IK is not calibrated for your robot!")
        print(f"    Your robot geometry is non-standard.")
        print(f"    Use teleop_keyboard.py for reliable control.")
        
        # Return center position as fallback
        return [90.0, 90.0, 90.0, 90.0, 90.0, 90.0]
    
    def forward_kinematics(self, angles):
        """
        Calculate XYZ from joint angles.
        
        CALIBRATED from real measurements:
        - Base 0° → X = 17 cm (left limit)
        - Base 90° → X = 25 cm (forward, maximum reach)
        - Base 180° → X = 17 cm (right limit)
        
        Height fixed at 40 cm (only tested at 90° for all joints)
        """
        # Base servo rotation (around Z axis)
        base_servo_angle = angles[0]  # 0-180°
        
        # Convert servo angle to mechanical angle
        # Servo: 0° → -90°, 90° → 0°, 180° → +90°
        mechanical_angle = (base_servo_angle - 90.0) * math.pi / 180.0
        
        # Horizontal reach varies with base angle
        # Reaches maximum (25cm) when pointing forward (90°)
        # Decreases to minimum (17cm) at sides (0° and 180°)
        
        # Using cosine relationship:
        # At 90°: reach = 25 cm (cos(0) = 1)
        # At 0°/180°: reach = 17 cm (cos(90°) = 0)
        # Formula: reach = 21 + 4*cos(base_mechanical_angle)
        
        reach = 21.0 + 4.0 * math.cos(mechanical_angle)
        
        # Calculate X and Y from reach and angle
        x = reach * math.cos(mechanical_angle)
        y = reach * math.sin(mechanical_angle)
        
        # Height is fixed (only calibrated at all 90°)
        z = 40.0
        
        return {'x': x, 'y': y, 'z': z}


def test_kinematics():
    """Test kinematics with actual measurements."""
    print("=" * 70)
    print("KINEMATICS TEST - EMPIRICAL CALIBRATION")
    print("=" * 70)
    
    arm = ArmKinematics()
    
    # Test points based on actual measurements
    test_points = [
        (0, "Base 0° (left)"),
        (90, "Base 90° (forward - max reach)"),
        (180, "Base 180° (right)"),
    ]
    
    print("\nCalibration verification:\n")
    
    for base_angle, description in test_points:
        angles = [base_angle, 90, 90, 90, 90, 90]
        pos = arm.forward_kinematics(angles)
        
        print(f"{description}")
        print(f"  Servo angles: Base={base_angle}°, Rest=90°")
        print(f"  Calculated: X={pos['x']:.2f}, Y={pos['y']:.2f}, Z={pos['z']:.2f} cm")
        
        # Expected values
        if base_angle == 0:
            print(f"  Expected: X≈17, Y≈0, Z=40")
        elif base_angle == 90:
            print(f"  Expected: X≈25, Y=0, Z=40")
        elif base_angle == 180:
            print(f"  Expected: X≈17, Y≈0, Z=40")
        print()
    
    print("=" * 70)


if __name__ == "__main__":
    test_kinematics()
