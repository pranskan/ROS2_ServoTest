"""
Inverse Kinematics for 6-DOF Robotic Arm
-----------------------------------------
Calculates joint angles to reach a target XYZ position.
"""
import math

class ArmKinematics:
    """
    Inverse kinematics solver for 6-DOF robotic arm.
    
    Arm structure (from base to gripper):
    - Joint 0 (Base): Rotates around vertical axis
    - Joint 1 (Shoulder): Rotates around horizontal axis
    - Joint 2 (Elbow): Rotates around horizontal axis
    - Joint 3 (Wrist Pitch): Rotates around horizontal axis
    - Joint 4 (Wrist Roll): Rotates around arm axis
    - Joint 5 (Gripper): Open/close
    """
    
    def __init__(self, link_lengths=None):
        """
        Initialize arm with link lengths (in mm or cm).
        
        Args:
            link_lengths: List of [L1, L2, L3, L4] lengths
                L1: Base to shoulder height
                L2: Shoulder to elbow length
                L3: Elbow to wrist length
                L4: Wrist to gripper length
        """
        if link_lengths is None:
            # Default link lengths (adjust for your arm)
            self.L1 = 10.0  # Base to shoulder height (cm)
            self.L2 = 15.0  # Shoulder to elbow length (cm)
            self.L3 = 15.0  # Elbow to wrist length (cm)
            self.L4 = 10.0  # Wrist to gripper length (cm)
        else:
            self.L1, self.L2, self.L3, self.L4 = link_lengths
    
    def inverse_kinematics(self, x, y, z, pitch=0, roll=0, gripper=90):
        """
        Calculate joint angles to reach target position.
        
        Args:
            x (float): Target X coordinate (cm)
            y (float): Target Y coordinate (cm)
            z (float): Target Z coordinate (cm)
            pitch (float): Desired gripper pitch angle (degrees)
            roll (float): Desired gripper roll angle (degrees)
            gripper (float): Gripper opening (0-180 degrees)
        
        Returns:
            list: [base, shoulder, elbow, wrist_pitch, wrist_roll, gripper]
                  or None if position is unreachable
        """
        
        # Joint 0 (Base): Calculate rotation around Z axis
        theta0 = math.degrees(math.atan2(y, x))
        
        # Distance from base to target in XY plane
        r = math.sqrt(x**2 + y**2)
        
        # Account for wrist length at desired pitch
        pitch_rad = math.radians(pitch)
        wx = self.L4 * math.cos(pitch_rad)
        wz = self.L4 * math.sin(pitch_rad)
        
        # Target position for wrist (end of arm, before gripper)
        target_r = r - wx
        target_z = z - self.L1 - wz
        
        # Distance from shoulder to wrist target
        d = math.sqrt(target_r**2 + target_z**2)
        
        # Check if target is reachable
        if d > (self.L2 + self.L3) or d < abs(self.L2 - self.L3):
            print(f"⚠️  Position unreachable: distance {d:.2f} cm")
            print(f"   Max reach: {self.L2 + self.L3:.2f} cm")
            print(f"   Min reach: {abs(self.L2 - self.L3):.2f} cm")
            return None
        
        # Joint 2 (Elbow): Use law of cosines
        cos_theta2 = (d**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        cos_theta2 = max(-1, min(1, cos_theta2))  # Clamp to valid range
        theta2 = math.degrees(math.acos(cos_theta2))
        
        # Joint 1 (Shoulder): Calculate shoulder angle
        alpha = math.atan2(target_z, target_r)
        beta = math.acos((self.L2**2 + d**2 - self.L3**2) / (2 * self.L2 * d))
        theta1 = math.degrees(alpha + beta)
        
        # Joint 3 (Wrist Pitch): Calculate to achieve desired end-effector pitch
        theta3 = pitch - (theta1 + theta2 - 180)
        
        # Joint 4 (Wrist Roll): Direct pass-through
        theta4 = roll
        
        # Joint 5 (Gripper): Direct pass-through
        theta5 = gripper
        
        # Convert to servo angles (0-180 range)
        angles = [
            self._normalize_angle(theta0 + 90),      # Base (center at 90°)
            self._normalize_angle(theta1),           # Shoulder
            self._normalize_angle(180 - theta2),     # Elbow (inverted)
            self._normalize_angle(theta3 + 90),      # Wrist pitch (center at 90°)
            self._normalize_angle(theta4 + 90),      # Wrist roll (center at 90°)
            self._normalize_angle(theta5)            # Gripper
        ]
        
        return angles
    
    def _normalize_angle(self, angle):
        """Normalize angle to 0-180 range."""
        return max(0, min(180, angle))
    
    def forward_kinematics(self, angles):
        """
        Calculate end-effector position from joint angles.
        
        Args:
            angles: [base, shoulder, elbow, wrist_pitch, wrist_roll, gripper]
        
        Returns:
            dict: {'x': x, 'y': y, 'z': z} position in cm
        """
        theta0, theta1, theta2, theta3, _, _ = [math.radians(a) for a in angles]
        
        # Adjust for servo center positions
        theta0 -= math.radians(90)
        theta2 = math.radians(180) - theta2
        theta3 -= math.radians(90)
        
        # Calculate cumulative angles
        a1 = theta1
        a2 = theta1 + theta2 - math.pi
        a3 = a2 + theta3
        
        # Calculate position
        r = (self.L2 * math.cos(a1) + 
             self.L3 * math.cos(a2) + 
             self.L4 * math.cos(a3))
        
        z = (self.L1 + 
             self.L2 * math.sin(a1) + 
             self.L3 * math.sin(a2) + 
             self.L4 * math.sin(a3))
        
        x = r * math.cos(theta0)
        y = r * math.sin(theta0)
        
        return {'x': x, 'y': y, 'z': z}


def test_kinematics():
    """Test the kinematics functions."""
    print("=" * 60)
    print("KINEMATICS TEST")
    print("=" * 60)
    print()
    
    # Create arm with default dimensions
    arm = ArmKinematics()
    
    print(f"Arm dimensions:")
    print(f"  L1 (Base height): {arm.L1} cm")
    print(f"  L2 (Upper arm):   {arm.L2} cm")
    print(f"  L3 (Forearm):     {arm.L3} cm")
    print(f"  L4 (Gripper):     {arm.L4} cm")
    print(f"  Max reach: {arm.L2 + arm.L3 + arm.L4:.2f} cm")
    print()
    
    # Test positions
    test_positions = [
        (20, 0, 30, "Forward"),
        (0, 20, 30, "Left"),
        (15, 15, 25, "Diagonal"),
        (10, 0, 40, "High"),
    ]
    
    print("Testing positions:")
    print("-" * 60)
    
    for x, y, z, description in test_positions:
        print(f"\n{description}: ({x}, {y}, {z}) cm")
        
        angles = arm.inverse_kinematics(x, y, z)
        
        if angles:
            print(f"  Joint angles:")
            servo_names = ["Base", "Shoulder", "Elbow", "Wrist Pitch", "Wrist Roll", "Gripper"]
            for i, (name, angle) in enumerate(zip(servo_names, angles)):
                print(f"    {i} ({name}): {angle:.1f}°")
            
            # Verify with forward kinematics
            pos = arm.forward_kinematics(angles)
            print(f"  Verification: ({pos['x']:.2f}, {pos['y']:.2f}, {pos['z']:.2f}) cm")
        else:
            print(f"  ✗ Position unreachable")
    
    print()
    print("=" * 60)


if __name__ == "__main__":
    test_kinematics()
