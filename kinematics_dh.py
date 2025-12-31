"""
Forward Kinematics using DH Parameters
Standard robotics method for calculating end-effector position
"""

import numpy as np
import math


class ArmKinematicsDH:
    """DH-based kinematics solver."""
    
    def __init__(self, base_height=10.5, l2=12.9, l3=11.0, l4=15.0):
        """
        Initialize with measured dimensions.
        
        Args:
            base_height: Z offset at base (cm)
            l2: Upper arm length (cm)
            l3: Forearm length (cm)
            l4: Gripper length (cm)
        """
        self.base_height = base_height
        self.l2 = l2
        self.l3 = l3
        self.l4 = l4
    
    def dh_matrix(self, a, alpha, d, theta):
        """
        Create homogeneous transformation matrix from DH parameters.
        
        Standard DH convention.
        """
        # Convert to radians
        alpha_rad = math.radians(alpha)
        theta_rad = math.radians(theta)
        
        cos_theta = math.cos(theta_rad)
        sin_theta = math.sin(theta_rad)
        cos_alpha = math.cos(alpha_rad)
        sin_alpha = math.sin(alpha_rad)
        
        # Standard DH transformation matrix
        T = np.array([
            [cos_theta, -sin_theta*cos_alpha, sin_theta*sin_alpha, a*cos_theta],
            [sin_theta, cos_theta*cos_alpha, -cos_theta*sin_alpha, a*sin_theta],
            [0, sin_alpha, cos_alpha, d],
            [0, 0, 0, 1]
        ], dtype=float)
        
        return T
    
    def forward_kinematics(self, angles):
        """
        Calculate end-effector position from joint angles.
        
        angles: [gripper(0), wrist_roll(1), wrist_pitch(2), elbow(3), shoulder(4), base(5)]
                (in degrees, 0-180)
        
        Returns: {'x': x_cm, 'y': y_cm, 'z': z_cm}
        
        At all 90°, the arm should be:
        - X = l2 + l3 + l4 = 12.9 + 11.0 + 15.0 = 38.9 cm (forward)
        - Y = 0 (centered)
        - Z = base_height = 10.5 cm (table height)
        """
        # Extract angles
        gripper = angles[0]
        wrist_roll = angles[1]
        wrist_pitch = angles[2]
        elbow = angles[3]
        shoulder = angles[4]
        base = angles[5]
        
        # Start with identity matrix
        T = np.eye(4)
        
        # Joint 1: Base rotation (rotates around Z axis)
        # This changes X and Y based on base angle
        # At 90°, should point forward (positive X)
        # At 0°, should point left (negative Y)
        # At 180°, should point right (positive Y)
        base_angle = base - 90.0  # Center at 90°
        T1 = self.dh_matrix(a=0, alpha=0, d=self.base_height, theta=base_angle)
        T = T @ T1
        
        # Joint 2: Shoulder pitch
        # At 90°, arm is horizontal (no Z change)
        # At 0°, arm points down
        # At 180°, arm points up
        shoulder_angle = shoulder - 90.0
        T2 = self.dh_matrix(a=self.l2, alpha=0, d=0, theta=shoulder_angle)
        T = T @ T2
        
        # Joint 3: Elbow pitch
        elbow_angle = elbow - 90.0
        T3 = self.dh_matrix(a=self.l3, alpha=0, d=0, theta=elbow_angle)
        T = T @ T3
        
        # Joint 4: Wrist pitch
        wrist_pitch_angle = wrist_pitch - 90.0
        T4 = self.dh_matrix(a=self.l4, alpha=0, d=0, theta=wrist_pitch_angle)
        T = T @ T4
        
        # Joint 5: Wrist roll (doesn't affect XYZ, only rotation)
        # Joint 6: Gripper (doesn't affect position, only opening)
        
        # Extract position
        x = T[0, 3]
        y = T[1, 3]
        z = T[2, 3]
        
        return {'x': x, 'y': y, 'z': z}
    
    def inverse_kinematics(self, x, y, z):
        """
        Solve inverse kinematics (XYZ → angles).
        
        This is complex for 6-DOF arms.
        For now, returns None (use forward kinematics only).
        """
        return None


def test_kinematics():
    """Test DH kinematics."""
    print("=" * 70)
    print("DH FORWARD KINEMATICS TEST")
    print("=" * 70)
    
    # Initialize with your measurements
    arm = ArmKinematicsDH(
        base_height=10.5,
        l2=12.9,
        l3=11.0,
        l4=15.0
    )
    
    # Test some positions
    test_cases = [
        ([90, 90, 90, 90, 90, 90], "All at 90° (neutral - arm forward)"),
        ([90, 90, 90, 90, 45, 90], "Shoulder at 45° (arm up)"),
        ([90, 90, 90, 90, 135, 90], "Shoulder at 135° (arm down)"),
        ([90, 90, 90, 90, 90, 0], "Base at 0° (arm left)"),
        ([90, 90, 90, 90, 90, 180], "Base at 180° (arm right)"),
    ]
    
    print("\nForward Kinematics Results:\n")
    print("Expected at 90°: X=38.9 cm, Y=0 cm, Z=10.5 cm\n")
    
    for angles, description in test_cases:
        pos = arm.forward_kinematics(angles)
        print(f"{description}")
        print(f"  Angles: {angles}")
        print(f"  Position: X={pos['x']:.2f}, Y={pos['y']:.2f}, Z={pos['z']:.2f} cm")
        print()
    
    print("=" * 70)


if __name__ == "__main__":
    test_kinematics()
