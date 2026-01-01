"""
DH-based Forward Kinematics
Apply correct DH theta offsets for all joints
"""

import math
import numpy as np

class ArmKinematicsDH:
    def __init__(self):
        """
        DH Parameters: [theta_DH, d (mm), a (mm), alpha (degrees)]
        theta is applied with -90° offset from command angles
        """
        # DH parameters for 4-DOF arm
        self.dh_params = [
            # Joint 1: theta1_DH = theta1_cmd - 90°
            {'d': 105, 'a': 0, 'alpha': 90, 'theta_offset': -90},
            
            # Joint 2: theta2_DH = theta2_cmd - 90°
            {'d': 0, 'a': 129, 'alpha': 0, 'theta_offset': -90},
            
            # Joint 3: theta3_DH = theta3_cmd - 90°
            {'d': 0, 'a': 110, 'alpha': 0, 'theta_offset': -90},
            
            # Joint 4: theta4_DH = theta4_cmd - 90°
            {'d': 0, 'a': 150, 'alpha': 0, 'theta_offset': -90},
        ]

    def dh_transform(self, theta_deg, d, a, alpha_deg):
        """
        Compute DH transformation matrix for a single joint.
        """
        theta = np.radians(theta_deg)
        alpha = np.radians(alpha_deg)
        
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,              np.sin(alpha),                np.cos(alpha),               d],
            [0,              0,                            0,                           1]
        ])

    def forward_kinematics(self, angles):
        """
        Compute end-effector position using DH parameters with theta offsets.
        
        angles: list of joint angles [theta1, theta2, theta3, theta4, ...]
        Returns: {'x': x_mm, 'y': y_mm, 'z': z_mm}
        """
        # Start with identity matrix
        T = np.eye(4)
        
        # Process first 4 active joints (ignore gripper/extra joints)
        for i in range(min(4, len(angles))):
            params = self.dh_params[i]
            # Apply theta offset
            theta_dh = angles[i] + params['theta_offset']
            
            # Compute DH transformation
            T_i = self.dh_transform(theta_dh, params['d'], params['a'], params['alpha'])
            T = T @ T_i
        
        # Extract position in mm
        x = T[0, 3]
        y = T[1, 3]
        z = T[2, 3]
        
        return {'x': x, 'y': y, 'z': z}