"""
Diagnose arm position at different angles
Help identify which servo is causing Z calculation to be wrong
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from kinematics_dh import ArmKinematicsDH
import time


class ArmDiagnostic(Node):
    def __init__(self):
        super().__init__('arm_diagnostic')
        self.arm_pub = self.create_publisher(Float32MultiArray, 'arm_command', 10)
        self.kinematics = ArmKinematicsDH(
            base_height=10.5,
            l2=12.9,
            l3=11.0,
            l4=15.0
        )
        time.sleep(2)
    
    def move_and_measure(self, angles, description):
        """Move to angles and calculate XYZ."""
        msg = Float32MultiArray()
        msg.data = angles
        
        # Publish
        for _ in range(10):
            self.arm_pub.publish(msg)
            time.sleep(0.1)
        
        time.sleep(2)
        
        # Calculate
        xyz = self.kinematics.forward_kinematics(angles)
        
        print(f"\n{description}")
        print(f"  Angles: {[f'{a:.0f}°' for a in angles]}")
        print(f"  Calculated XYZ: X={xyz['x']:6.2f}, Y={xyz['y']:6.2f}, Z={xyz['z']:6.2f} cm")
        
        # Ask user
        user_z = input(f"  What is the measured Z height? (cm): ")
        try:
            measured_z = float(user_z)
            error = measured_z - xyz['z']
            print(f"  ERROR: {error:+6.2f} cm (measured {measured_z:.2f} - calculated {xyz['z']:.2f})")
        except:
            print("  (skipped measurement)")
        
        return xyz


def main():
    rclpy.init()
    node = ArmDiagnostic()
    
    print("=" * 70)
    print("ARM POSITION DIAGNOSTIC")
    print("=" * 70)
    print("\nFor each position, the arm will move there.")
    print("Then MEASURE the Z height (distance from table to gripper)")
    print("Enter the actual measurement to compare with calculated value.\n")
    
    try:
        # Test 1: All at center (default)
        node.move_and_measure(
            [90.0, 90.0, 135.0, 135.0, 135.0, 135.0],
            "TEST 1: All motors at CENTER position"
        )
        
        # Test 2: Shoulder pitched up
        node.move_and_measure(
            [90.0, 90.0, 135.0, 135.0, 45.0, 135.0],
            "TEST 2: Shoulder pitched UP (45°)"
        )
        
        # Test 3: Shoulder pitched down
        node.move_and_measure(
            [90.0, 90.0, 135.0, 135.0, 225.0, 135.0],
            "TEST 3: Shoulder pitched DOWN (225°)"
        )
        
        # Test 4: Elbow pitched up
        node.move_and_measure(
            [90.0, 90.0, 135.0, 45.0, 135.0, 135.0],
            "TEST 4: Elbow pitched UP (45°)"
        )
        
        # Test 5: Just base height
        node.move_and_measure(
            [90.0, 90.0, 90.0, 90.0, 90.0, 135.0],
            "TEST 5: All 90° (should be low if 90° is 'down')"
        )
        
        print("\n" + "=" * 70)
        print("ANALYSIS:")
        print("=" * 70)
        print("""
If Z doesn't change when you move shoulder/elbow:
  → The kinematics isn't accounting for those joints

If Z is always 10.5 (just base height):
  → Shoulder and elbow angles aren't affecting Z
  → Check if 135° is actually the center position
  → Or check if servos are physically moving

If Z changes but calculations are wrong:
  → The center position (135°) might be incorrect
  → Or the DH parameters need adjustment
        """)
        
    except KeyboardInterrupt:
        print("\n\nStopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
