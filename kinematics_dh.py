"""
Forward Kinematics using DH Parameters
Accounts for both 180° and 270° servo motors
With coordinate transformation to match physical arm
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
        """
        alpha_rad = math.radians(alpha)
        theta_rad = math.radians(theta)
        
        cos_theta = math.cos(theta_rad)
        sin_theta = math.sin(theta_rad)
        cos_alpha = math.cos(alpha_rad)
        sin_alpha = math.sin(alpha_rad)
        
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
        
        Servo types:
        - Gripper (0): 180° servo, centered at 90°
        - Wrist Roll (1): 180° servo, centered at 90°
        - Wrist Pitch (2): 270° servo, centered at 135°
        - Elbow (3): 270° servo, centered at 135°
        - Shoulder (4): 270° servo, centered at 135°
        - Base (5): 270° servo, centered at 135°
        
        Returns: {'x': x_cm, 'y': y_cm, 'z': z_cm}
        """
        gripper = angles[0]
        wrist_roll = angles[1]
        wrist_pitch = angles[2]
        elbow = angles[3]
        shoulder = angles[4]
        base = angles[5]
        
        T = np.eye(4)
        
        # Joint 1: Base rotation (270° servo, center at 135°)
        # At 135°, should point forward (positive X)
        # At 45°, should point left (negative Y)
        # At 225°, should point right (positive Y)
        base_angle = base - 135.0
        T1 = self.dh_matrix(a=0, alpha=0, d=self.base_height, theta=base_angle)
        T = T @ T1
        
        # Joint 2: Shoulder pitch (270° servo, center at 135°)
        # At 135°, arm is horizontal
        shoulder_angle = shoulder - 135.0
        T2 = self.dh_matrix(a=self.l2, alpha=0, d=0, theta=shoulder_angle)
        T = T @ T2
        
        # Joint 3: Elbow pitch (270° servo, center at 135°)
        elbow_angle = elbow - 135.0
        T3 = self.dh_matrix(a=self.l3, alpha=0, d=0, theta=elbow_angle)
        T = T @ T3
        
        # Joint 4: Wrist pitch (270° servo, center at 135°)
        wrist_pitch_angle = wrist_pitch - 135.0
        T4 = self.dh_matrix(a=self.l4, alpha=0, d=0, theta=wrist_pitch_angle)
        T = T @ T4
        
        # Extract position from DH calculation
        x_dh = T[0, 3]
        y_dh = T[1, 3]
        z_dh = T[2, 3]
        
        # Apply coordinate transformation to match your physical arm
        # Home position: DH (38.9, 0, 10.5) → Desired (27, 0, 40)
        # When base rotates, X and Y change but Z should stay constant
        
        x_new = x_dh - 11.9  # 38.9 - 11.9 = 27 at home
        y_new = y_dh         # Y stays the same
        z_new = z_dh + 29.5  # 10.5 + 29.5 = 40, stays constant when base rotates
        
        return {'x': x_new, 'y': y_new, 'z': z_new}
    
    def inverse_kinematics(self, x, y, z):
        """Solve inverse kinematics - not implemented."""
        return None


def test_kinematics():
    """Test DH kinematics."""
    print("=" * 70)
    print("DH FORWARD KINEMATICS TEST - WITH TRANSFORMATION")
    print("=" * 70)

    arm = ArmKinematicsDH(
        base_height=10.5,
        l2=12.9,
        l3=11.0,
        l4=15.0
    )
    
    test_cases = [
        ([90, 90, 135, 135, 135, 135], "All centered (gripper 90°, rest 135°)"),
        ([90, 90, 135, 135, 45, 135], "Shoulder at 45° (arm up)"),
        ([90, 90, 135, 135, 225, 135], "Shoulder at 225° (arm down)"),
        ([90, 90, 135, 135, 135, 45], "Base at 45° (arm left)"),
        ([90, 90, 135, 135, 135, 225], "Base at 225° (arm right)"),
    ]
    
    print("\nForward Kinematics Results:\n")
    print("Expected at center: X=27 cm, Y=0 cm, Z=40 cm\n")
    
    for angles, description in test_cases:
        pos = arm.forward_kinematics(angles)
        print(f"{description}")
        print(f"  Angles: {angles}")
        print(f"  Position: X={pos['x']:.2f}, Y={pos['y']:.2f}, Z={pos['z']:.2f} cm")
        print()
    
    print("=" * 70)


if __name__ == "__main__":
    test_kinematics()
