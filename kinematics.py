"""
Inverse Kinematics for 6-DOF Robotic Arm
-----------------------------------------
Calculates joint angles to reach a target XYZ position.

COORDINATE SYSTEM:
- X: Forward/backward (positive = forward from base)
- Y: Left/right (positive = left when facing forward)
- Z: Up/down (positive = up from ground)
- Origin: Center of base rotation

LINK LENGTHS (L1, L2, L3, L4):
- L1: Vertical distance from ground to shoulder joint (base height)
- L2: Distance from shoulder to elbow (upper arm length)
- L3: Distance from elbow to wrist (forearm length)
- L4: Distance from wrist to gripper tip (end-effector length)

WORKSPACE LIMITS:
- Max horizontal reach: L2 + L3 + L4 (all links extended)
- Min horizontal reach: |L2 - L3| (elbow fully bent)
- Max height: L1 + L2 + L3 + L4 (fully extended up)
- Min height: L1 - (L2 + L3 + L4) (fully extended down)

EXAMPLE:
With default values (L1=10, L2=15, L3=15, L4=10):
- Max reach: 40 cm horizontal
- Height range: -30 cm to 50 cm
- Total workspace: Cylindrical volume around base
"""
import math

class ArmKinematics:
    """
    Inverse kinematics solver for 6-DOF robotic arm.
    
    JOINT DEFINITIONS:
    - Joint 0 (Base):        Rotates around Z-axis (vertical)
                             Range: 0-180° (90° = forward)
    - Joint 1 (Shoulder):    Rotates around Y-axis (horizontal)
                             Range: 0-180° (90° = horizontal forward)
    - Joint 2 (Elbow):       Rotates around Y-axis (horizontal)
                             Range: 0-180° (90° = straight, 0° = fully bent)
    - Joint 3 (Wrist Pitch): Rotates around Y-axis (horizontal)
                             Range: 0-180° (90° = horizontal)
    - Joint 4 (Wrist Roll):  Rotates around X-axis (arm length)
                             Range: 0-180° (90° = neutral)
    - Joint 5 (Gripper):     Open/close mechanism
                             Range: 0-180° (0° = closed, 180° = open)
    """
    
    def __init__(self, link_lengths=None):
        """
        Initialize arm with link lengths.
        
        Args:
            link_lengths: List of [L1, L2, L3, L4] lengths in cm
        """
        if link_lengths is None:
            # YOUR ROBOT'S ACTUAL MEASUREMENTS (in cm)
            self.L1 = 10.5  # Base to shoulder height: 105mm
            self.L2 = 12.9  # Shoulder to elbow length: 129mm
            self.L3 = 11.0  # Elbow to wrist length: 110mm
            self.L4 = 15.0  # Wrist to gripper tip: 150mm
        else:
            self.L1, self.L2, self.L3, self.L4 = link_lengths
        
        # Calculate workspace limits
        self.max_horizontal_reach = self.L2 + self.L3 + self.L4
        self.min_horizontal_reach = abs(self.L2 - self.L3)
        self.max_height = self.L1 + self.L2 + self.L3 + self.L4
        self.min_height = self.L1 - (self.L2 + self.L3 + self.L4)
        
        # DON'T print config during init - only when explicitly called
        # self._print_config()  # REMOVED
    
    def _print_config(self):
        """Print arm configuration and limits."""
        print("\n" + "=" * 60)
        print("ROBOTIC ARM CONFIGURATION")
        print("=" * 60)
        print(f"\nLink Lengths (cm):")
        print(f"  L1 (Base height):       {self.L1:6.2f} cm")
        print(f"  L2 (Upper arm):         {self.L2:6.2f} cm")
        print(f"  L3 (Forearm):           {self.L3:6.2f} cm")
        print(f"  L4 (Gripper):           {self.L4:6.2f} cm")
        print(f"  Total arm length:       {self.L2 + self.L3 + self.L4:6.2f} cm")
        print(f"\nWorkspace Limits:")
        print(f"  Max horizontal reach:   {self.max_horizontal_reach:6.2f} cm")
        print(f"  Min horizontal reach:   {self.min_horizontal_reach:6.2f} cm")
        print(f"  Max height (Z):         {self.max_height:6.2f} cm")
        print(f"  Min height (Z):         {self.min_height:6.2f} cm")
        print(f"\nCoordinate System:")
        print(f"  X: Forward/back  (+ = forward)")
        print(f"  Y: Left/right    (+ = left)")
        print(f"  Z: Up/down       (+ = up)")
        print(f"  Origin: Base center at ground level")
        print("=" * 60 + "\n")
    
    def get_workspace_limits(self):
        """
        Get workspace limits for the robotic arm.
        
        Returns:
            dict: Workspace limits
                {
                    'x_max': float,  # Max X coordinate (cm)
                    'x_min': float,  # Min X coordinate (cm)
                    'y_max': float,  # Max Y coordinate (cm)
                    'y_min': float,  # Min Y coordinate (cm)
                    'z_max': float,  # Max Z coordinate (cm)
                    'z_min': float,  # Min Z coordinate (cm)
                    'max_reach': float,  # Max horizontal reach (cm)
                    'min_reach': float,  # Min horizontal reach (cm)
                }
        """
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
        """
        Check if a position is within the robot's workspace.
        
        Args:
            x, y, z: Target position (cm)
        
        Returns:
            tuple: (reachable: bool, reason: str)
        """
        # Check height limits
        if z > self.max_height:
            return False, f"Too high: {z:.2f} > {self.max_height:.2f} cm"
        if z < self.min_height:
            return False, f"Too low: {z:.2f} < {self.min_height:.2f} cm"
        
        # Check horizontal reach
        r = math.sqrt(x**2 + y**2)
        if r > self.max_horizontal_reach:
            return False, f"Too far: {r:.2f} > {self.max_horizontal_reach:.2f} cm"
        if r < self.min_horizontal_reach:
            return False, f"Too close: {r:.2f} < {self.min_horizontal_reach:.2f} cm"
        
        return True, "Position reachable"
    
    def inverse_kinematics(self, x, y, z, pitch=0, roll=0, gripper=90):
        """
        Calculate joint angles to reach target position.
        
        Args:
            x (float): Target X coordinate (cm)
            y (float): Target Y coordinate (cm)
            z (float): Target Z coordinate (cm)
            pitch (float): Desired gripper pitch angle (degrees, default 0 = horizontal)
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
        
        # SIMPLIFIED: For horizontal gripper (pitch=0), the wrist is at the target
        # We'll solve for wrist position = target position
        # This matches what teleop does (all joints affect position directly)
        target_r = r
        target_z = z - self.L1  # Subtract base height
        
        # Distance from shoulder to target (accounting for wrist and gripper)
        # When all joints are horizontal, we need: shoulder->elbow->wrist->gripper
        # But the target IS the gripper tip, so we work backwards
        wrist_r = target_r - self.L4  # Gripper extends horizontally
        wrist_z = target_z
        
        # Distance from shoulder to wrist
        d = math.sqrt(wrist_r**2 + wrist_z**2)
        
        # Check if wrist position is reachable by shoulder+elbow
        if d > (self.L2 + self.L3) or d < abs(self.L2 - self.L3):
            print(f"⚠️  Position unreachable: distance {d:.2f} cm")
            print(f"   Max reach: {self.L2 + self.L3:.2f} cm")
            print(f"   Min reach: {abs(self.L2 - self.L3):.2f} cm")
            print(f"   (This is shoulder->elbow->wrist distance, not including gripper)")
            return None
        
        # Joint 2 (Elbow): Use law of cosines
        cos_theta2 = (d**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        cos_theta2 = max(-1, min(1, cos_theta2))  # Clamp to valid range
        theta2 = math.degrees(math.acos(cos_theta2))
        
        # Joint 1 (Shoulder): Calculate shoulder angle
        alpha = math.atan2(wrist_z, wrist_r)
        beta = math.acos((self.L2**2 + d**2 - self.L3**2) / (2 * self.L2 * d))
        theta1 = math.degrees(alpha + beta)
        
        # Joint 3 (Wrist Pitch): Keep gripper horizontal
        # The wrist needs to compensate for shoulder and elbow angles
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
    
    # Create arm with default dimensions
    arm = ArmKinematics()
    
    # Get workspace limits
    limits = arm.get_workspace_limits()
    print("\nWorkspace Summary:")
    print(f"  X range: {limits['x_min']:.1f} to {limits['x_max']:.1f} cm")
    print(f"  Y range: {limits['y_min']:.1f} to {limits['y_max']:.1f} cm")
    print(f"  Z range: {limits['z_min']:.1f} to {limits['z_max']:.1f} cm")
    print()
    
    # Test positions
    test_positions = [
        (20, 0, 30, "Forward, medium height"),
        (0, 20, 30, "Left side, medium height"),
        (15, 15, 25, "Diagonal forward-left"),
        (10, 0, 40, "Forward, high"),
        (30, 0, 20, "Far forward, low"),
        (50, 0, 30, "Too far (should fail)"),
    ]
    
    print("Testing positions:")
    print("-" * 60)
    
    for x, y, z, description in test_positions:
        print(f"\n{description}: X={x}, Y={y}, Z={z} cm")
        
        # Check if reachable
        reachable, reason = arm.is_position_reachable(x, y, z)
        if not reachable:
            print(f"  ✗ {reason}")
            continue
        
        angles = arm.inverse_kinematics(x, y, z)
        
        if angles:
            print(f"  ✓ Solution found:")
            servo_names = ["Base", "Shoulder", "Elbow", "Wrist Pitch", "Wrist Roll", "Gripper"]
            for i, (name, angle) in enumerate(zip(servo_names, angles)):
                print(f"    Joint {i} ({name:12s}): {angle:6.1f}°")
            
            # Verify with forward kinematics
            pos = arm.forward_kinematics(angles)
            error_x = abs(pos['x'] - x)
            error_y = abs(pos['y'] - y)
            error_z = abs(pos['z'] - z)
            print(f"  Verification: X={pos['x']:.2f}, Y={pos['y']:.2f}, Z={pos['z']:.2f} cm")
            print(f"  Error: {error_x:.3f}, {error_y:.3f}, {error_z:.3f} cm")
        else:
            print(f"  ✗ No solution found")
    
    print("\n" + "=" * 60)
    print("\nTO CUSTOMIZE FOR YOUR ROBOT:")
    print("1. Measure your link lengths (L1, L2, L3, L4)")
    print("2. Update in: arm_kinematics = ArmKinematics([L1, L2, L3, L4])")
    print("3. Run this test again to verify workspace")
    print("=" * 60 + "\n")

#12dfddd
if __name__ == "__main__":
    test_kinematics()
