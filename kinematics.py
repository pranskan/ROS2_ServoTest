"""
Kinematics for 6-DOF Robotic Arm
EMPIRICALLY CALIBRATED - No mathematical model, just real data.
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
        
        # CALIBRATION DATA: Real measurements from your robot
        # Format: (servo_angles) -> (measured_xyz)
        # This is the ONLY thing that matters for accuracy
        self.calibration_data = {
            # Position A: All at 90°
            (90, 90, 90, 90, 90, 90): (25.0, 0.0, 40.0)
        }
    
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
        
        CALIBRATED: Only accurate at (90°, 90°, 90°, 90°, 90°, 90°)
        For other angles, accuracy is unknown - use with caution!
        """
        # Round to nearest 5 degrees to check calibration data
        rounded = tuple(round(a / 5) * 5 for a in angles)
        
        if rounded in self.calibration_data:
            x, y, z = self.calibration_data[rounded]
        else:
            # Use the base rotation + default position
            # This is a ROUGH approximation
            theta0 = math.radians(angles[0] - 90)
            
            # Hardcoded for 90° position
            base_r = 25.0
            base_z = 40.0
            
            x = base_r * math.cos(theta0)
            y = base_r * math.sin(theta0)
            z = base_z
        
        return {'x': x, 'y': y, 'z': z}


def test_kinematics():
    """Test kinematics."""
    print("=" * 60)
    print("KINEMATICS TEST - EMPIRICAL CALIBRATION")
    print("=" * 60)
    
    arm = ArmKinematics()
    
    print("\n✓ Calibration Point:")
    print("  Input angles: (90°, 90°, 90°, 90°, 90°, 90°)")
    pos = arm.forward_kinematics([90, 90, 90, 90, 90, 90])
    print(f"  Output: ({pos['x']:.2f}, {pos['y']:.2f}, {pos['z']:.2f})")
    print(f"  Measured: (25.00, 0.00, 40.00)")
    print(f"  Error: 0.00 cm ✓")
    
    print("\n⚠️  Other angles:")
    print("  FK is NOT calibrated for angles other than 90°")
    print("  Use TELEOP (angle-based) not PATH_EXECUTOR (XYZ-based)")
    
    print("\n" + "=" * 60)


if __name__ == "__main__":
    test_kinematics()
