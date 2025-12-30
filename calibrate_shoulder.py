"""
Calibrate Shoulder Joint
Keep base at 90° (pointing forward), vary shoulder angle
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time


class ShoulderCalibrator(Node):
    def __init__(self):
        super().__init__('shoulder_calibrator')
        self.arm_pub = self.create_publisher(Float32MultiArray, 'arm_command', 10)
        time.sleep(2)
        self.get_logger().info('Ready to calibrate shoulder')
    
    def measure_shoulder(self):
        """Measure Z height at different shoulder angles."""
        print("\n" + "=" * 70)
        print("SHOULDER CALIBRATION")
        print("=" * 70)
        print("\nBase at 90° (pointing forward)")
        print("Vary shoulder angle and measure Z height\n")
        
        measurements = []
        test_angles = [0, 30, 60, 90, 120, 150, 180]
        
        for shoulder_angle in test_angles:
            # Base=90°, Shoulder=variable, rest=90°
            angles = [90.0, 90.0, 90.0, 90.0, shoulder_angle, 90.0]
            
            print("=" * 70)
            print(f"MEASURING: Shoulder at {shoulder_angle}°")
            print("=" * 70)
            
            msg = Float32MultiArray()
            msg.data = angles
            
            for _ in range(5):
                self.arm_pub.publish(msg)
                time.sleep(0.1)
            
            time.sleep(2)
            
            print(f"\n📏 Measure the arm position:")
            try:
                x = float(input("  X position (cm): "))
                y = float(input("  Y position (cm): "))
                z = float(input("  Z position (cm): "))
            except ValueError:
                print("Invalid input!")
                continue
            
            measurements.append({
                'shoulder': shoulder_angle,
                'x': x, 'y': y, 'z': z
            })
            print()
        
        return measurements
    
    def analyze_shoulder(self, measurements):
        """Analyze shoulder measurements."""
        print("\n" + "=" * 70)
        print("SHOULDER ANALYSIS")
        print("=" * 70)
        print("\nMeasurements (Base=90°, Shoulder variable):\n")
        
        for m in measurements:
            print(f"Shoulder {m['shoulder']:3.0f}° → X={m['x']:6.2f}, Y={m['y']:6.2f}, Z={m['z']:6.2f}")
        
        # Analyze Z vs shoulder angle
        z_values = [m['z'] for m in measurements]
        shoulder_angles = [m['shoulder'] for m in measurements]
        
        print(f"\nZ height range: {min(z_values):.2f} to {max(z_values):.2f} cm")
        print(f"X range: {min([m['x'] for m in measurements]):.2f} to {max([m['x'] for m in measurements]):.2f} cm")
        
        print("\nUse these values to update kinematics.py forward_kinematics()")


def main():
    rclpy.init()
    node = ShoulderCalibrator()
    
    try:
        measurements = node.measure_shoulder()
        node.analyze_shoulder(measurements)
    except KeyboardInterrupt:
        print("\n\nStopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
