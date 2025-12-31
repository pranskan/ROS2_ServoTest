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
        
        # DH Parameters table
        # [a, alpha, d, theta_offset]
        self.dh_table = [
            [0,    0,    base_height, 0],      # Joint 1: Base
            [0,   90,    0,           90],     # Joint 2: Shoulder
            [l2,   0,    0,           0],      # Joint 3: Upper arm
            [0,   90,    0,           0],      # Joint 4: Elbow
            [l3,   0,    0,           0],      # Joint 5: Forearm
            [0,   90,    0,           0],      # Joint 6: Wrist
            [l4,   0,    0,           0],      # End effector
        ]
    
    def dh_matrix(self, a, alpha, d, theta):
        """
        Create homogeneous transformation matrix from DH parameters.
        
        T = Tz(d) * Rx(alpha) * Tx(a) * Rz(theta)
        """
        # Convert to radians
        alpha_rad = math.radians(alpha)
        theta_rad = math.radians(theta)
        
        # Build transformation matrix
        T = np.array([
            [math.cos(theta_rad), -math.sin(theta_rad)*math.cos(alpha_rad), 
             math.sin(theta_rad)*math.sin(alpha_rad), a*math.cos(theta_rad)],
            
            [math.sin(theta_rad), math.cos(theta_rad)*math.cos(alpha_rad),
             -math.cos(theta_rad)*math.sin(alpha_rad), a*math.sin(theta_rad)],
            
            [0, math.sin(alpha_rad), math.cos(alpha_rad), d],
            [0, 0, 0, 1]
        ])
        
        return T
    
    def forward_kinematics(self, angles):
        """
        Calculate end-effector position from joint angles.
        
        angles: [base, shoulder, elbow, wrist_pitch, wrist_roll, gripper]
                (in degrees, 0-180)
        
        Returns: {'x': x_cm, 'y': y_cm, 'z': z_cm}
        """
        # Convert servo angles to DH angles
        # Servo 90° = joint center position
        dh_angles = [
            angles[5],           # Base
            angles[4] - 90,      # Shoulder (offset from 90°)
            angles[3] - 90,      # Elbow
            angles[2] - 90,      # Wrist pitch
            angles[1],           # Wrist roll
            angles[0],           # Gripper (not used in FK)
        ]
        
        # Calculate transformation matrix for each joint
        T = np.eye(4)  # Start with identity matrix
        
        for i in range(len(dh_angles)):
            a, alpha, d, theta_offset = self.dh_table[i]
            theta = dh_angles[i] + theta_offset
            
            # Get DH matrix for this joint
            T_i = self.dh_matrix(a, alpha, d, theta)
            
            # Multiply matrices (accumulate transformations)
            T = T @ T_i
        
        # Extract position from final transformation matrix
        x = T[0, 3]  # X position
        y = T[1, 3]  # Y position
        z = T[2, 3]  # Z position
        
        return {'x': x, 'y': y, 'z': z}
    
    def inverse_kinematics(self, x, y, z):
        """
        Solve inverse kinematics (XYZ → angles).
        
        This is complex for 6-DOF arms.
        For now, returns None (use forward kinematics only).
        """
        # IK for 6-DOF is very complex
        # Would need numerical solver or analytical solution
        return None


def test_kinematics():
    """Test DH kinematics."""
    print("=" * 70)
    print("DH FORWARD KINEMATICS TEST")
    print("=" * 70)
    
    # Initialize with your measurements
    arm = ArmKinematicsDH(
        base_height=10.5,  # Update with your measurement
        l2=12.9,           # Update with your measurement
        l3=11.0,           # Update with your measurement
        l4=15.0            # Update with your measurement
    )
    
    # Test some positions
    test_cases = [
        ([90, 90, 90, 90, 90, 90], "All at 90° (neutral)"),
        ([0, 90, 90, 90, 90, 90], "Base rotated left"),
        ([180, 90, 90, 90, 90, 90], "Base rotated right"),
        ([90, 45, 90, 90, 90, 90], "Shoulder pitched up"),
    ]
    
    print("\nForward Kinematics Results:\n")
    
    for angles, description in test_cases:
        pos = arm.forward_kinematics(angles)
        print(f"{description}")
        print(f"  Angles: {angles}")
        print(f"  Position: X={pos['x']:.2f}, Y={pos['y']:.2f}, Z={pos['z']:.2f} cm")
        print()
    
    print("=" * 70)


if __name__ == "__main__":
    test_kinematics()
