"""
Full FK/IK Calibration System
-----------------------------
Systematically moves your robot through workspace and collects XYZ measurements.
This data will be used to build accurate forward and inverse kinematics.

INSTRUCTIONS:
1. Start servo_control.py in one terminal
2. Run this script
3. When arm moves to each position, MEASURE with ruler and enter value
4. Script will build calibration model
5. Update kinematics.py with results

Time required: ~30 minutes
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
import json
from pathlib import Path


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        self.arm_pub = self.create_publisher(Float32MultiArray, 'arm_command', 10)
        self.calibration_points = []
        self.get_logger().info('Calibration Node Ready')
    
    def move_to_angles(self, angles, wait_time=2.0):
        """Move arm to specified angles and wait."""
        msg = Float32MultiArray()
        msg.data = angles
        self.arm_pub.publish(msg)
        time.sleep(wait_time)
    
    def collect_point(self, angles, description):
        """Collect one calibration point."""
        print("\n" + "=" * 70)
        print(f"CALIBRATION POINT: {description}")
        print("=" * 70)
        
        # Move to position
        print(f"Moving arm to: {[f'{a:.0f}°' for a in angles]}")
        self.move_to_angles(angles, wait_time=3.0)
        
        print("\n📏 MEASURE WITH RULER:")
        print("-" * 70)
        
        # Get measurements
        try:
            h_reach = float(input("Horizontal distance from base to gripper (cm): "))
            height = float(input("Vertical height from ground to gripper (cm): "))
        except ValueError:
            print("Invalid input! Skipping this point.")
            return None
        
        # Store data
        data_point = {
            'angles': list(angles),
            'horizontal': h_reach,
            'height': height
        }
        
        self.calibration_points.append(data_point)
        
        print(f"\n✓ Recorded: H={h_reach}cm, Z={height}cm")
        print(f"  Calibration points collected: {len(self.calibration_points)}")
        
        return data_point
    
    def run_calibration(self):
        """Run full calibration sequence."""
        print("\n" + "=" * 70)
        print("FULL CALIBRATION SEQUENCE")
        print("=" * 70)
        print("\nThis will collect calibration points across your robot's workspace.")
        print("For each position, measure horizontal reach and height with a ruler.")
        print("\nNote: Horizontal = distance from base center to gripper")
        print("      Height = distance from ground to gripper tip")
        
        time.sleep(2)
        
        # Define calibration points
        # Base angles: various horizontal directions
        # Shoulder angles: from low to high
        # Elbow angles: various configurations
        
        calibration_sequence = [
            # Format: (angles, description)
            
            # Basic reference points (3 positions)
            ([90, 90, 90, 90, 90, 90], "CENTER: All at 90°"),
            ([90, 60, 90, 90, 90, 90], "Shoulder at 60°"),
            ([90, 120, 90, 90, 90, 90], "Shoulder at 120°"),
            
            # Different Base angles (5 positions)
            ([60, 90, 90, 90, 90, 90], "Base at 60°, Rest at 90°"),
            ([120, 90, 90, 90, 90, 90], "Base at 120°, Rest at 90°"),
            
            # Different Elbow angles (3 positions)
            ([90, 90, 60, 90, 90, 90], "Elbow at 60°, Rest at 90°"),
            ([90, 90, 120, 90, 90, 90], "Elbow at 120°, Rest at 90°"),
            
            # Wrist Pitch variations (2 positions)
            ([90, 90, 90, 60, 90, 90], "Wrist Pitch at 60°, Rest at 90°"),
            ([90, 90, 90, 120, 90, 90], "Wrist Pitch at 120°, Rest at 90°"),
            
            # Combined movements (5 positions)
            ([90, 45, 90, 90, 90, 90], "Shoulder at 45°"),
            ([90, 135, 90, 90, 90, 90], "Shoulder at 135°"),
            ([90, 90, 45, 90, 90, 90], "Elbow at 45°"),
            ([90, 90, 135, 90, 90, 90], "Elbow at 135°"),
            ([90, 75, 75, 90, 90, 90], "Shoulder & Elbow at 75°"),
        ]
        
        print(f"\nPlanned {len(calibration_sequence)} calibration points")
        print("Press Enter to start measuring each point.\n")
        
        collected = 0
        for i, (angles, desc) in enumerate(calibration_sequence, 1):
            print(f"\n[{i}/{len(calibration_sequence)}] Press Enter to move to next position...")
            input()
            
            point = self.collect_point(angles, desc)
            if point:
                collected += 1
            
            # Ask to continue
            if i < len(calibration_sequence):
                cont = input("\nContinue? (y/n): ").lower()
                if cont != 'y':
                    print("Calibration stopped by user.")
                    break
        
        print("\n" + "=" * 70)
        print(f"✓ CALIBRATION COMPLETE!")
        print(f"  Collected {collected}/{len(calibration_sequence)} points")
        print("=" * 70)
        
        return self.calibration_points
    
    def save_data(self, filename='calibration_data.json'):
        """Save calibration data to file."""
        with open(filename, 'w') as f:
            json.dump(self.calibration_points, f, indent=2)
        print(f"\n✓ Data saved to {filename}")


def analyze_calibration(data):
    """Analyze collected calibration data."""
    print("\n" + "=" * 70)
    print("CALIBRATION ANALYSIS")
    print("=" * 70)
    
    print(f"\nCollected {len(data)} calibration points\n")
    
    # Find range of values
    base_angles = [p['angles'][5] for p in data]
    shoulder_angles = [p['angles'][4] for p in data]
    elbow_angles = [p['angles'][3] for p in data]
    wrist_angles = [p['angles'][2] for p in data]
    
    h_reach = [p['horizontal'] for p in data]
    heights = [p['height'] for p in data]
    
    print("Angle Ranges:")
    print(f"  Base: {min(base_angles):.0f}° - {max(base_angles):.0f}°")
    print(f"  Shoulder: {min(shoulder_angles):.0f}° - {max(shoulder_angles):.0f}°")
    print(f"  Elbow: {min(elbow_angles):.0f}° - {max(elbow_angles):.0f}°")
    print(f"  Wrist: {min(wrist_angles):.0f}° - {max(wrist_angles):.0f}°")
    
    print("\nPosition Ranges:")
    print(f"  Horizontal reach: {min(h_reach):.1f} - {max(h_reach):.1f} cm")
    print(f"  Height: {min(heights):.1f} - {max(heights):.1f} cm")
    
    print("\n" + "=" * 70)
    print("NEXT STEPS:")
    print("=" * 70)
    print("\n1. Data has been saved to calibration_data.json")
    print("2. I will analyze this data")
    print("3. I will generate updated kinematics.py with RBF interpolation")
    print("4. Upload calibration_data.json for me to process")
    print("\n" + "=" * 70)


def main():
    print("\n" + "=" * 70)
    print("FULL ROBOTIC ARM CALIBRATION")
    print("=" * 70)
    print("\nMake sure servo_control.py is running and the arm is safe to move!")
    print("\nPress Enter to start...")
    input()
    
    rclpy.init()
    node = CalibrationNode()
    
    # Run calibration
    time.sleep(1.0)
    calibration_data = node.run_calibration()
    
    # Analyze
    analyze_calibration(calibration_data)
    
    # Save
    node.save_data()
    
    node.destroy_node()
    rclpy.shutdown()
    
    print("\n✓ Calibration complete!")
    print("Share calibration_data.json with me to get updated kinematics.py")


if __name__ == '__main__':
    main()
