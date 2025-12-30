"""
Measure Actual Servo Physical Limits
-------------------------------------
Find the real min/max positions for each servo.

Your observation:
- Base at 0° → X = 17 cm (not -25 cm)
- Base at 180° → X = 17 cm (not +25 cm)
- Base at 90° → X = 25 cm (correct!)

This tells us the base servo has limited range!
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time


class ServoLimitsMeasurer(Node):
    def __init__(self):
        super().__init__('servo_limits_node')
        self.arm_pub = self.create_publisher(Float32MultiArray, 'arm_command', 10)
        time.sleep(2)  # Wait for connection
        self.get_logger().info('Ready to measure servo limits')
    
    def move_servo(self, servo_channel, angle, description=""):
        """Move a single servo and wait for user measurement."""
        print("\n" + "=" * 70)
        print(f"MEASURING: {description}")
        print("=" * 70)
        
        # Create command with all servos at 90° except the one we're testing
        angles = [90.0] * 6
        angles[servo_channel] = angle
        
        print(f"Moving servo {servo_channel} to {angle}°...")
        msg = Float32MultiArray()
        msg.data = angles
        
        # Publish multiple times
        for _ in range(5):
            self.arm_pub.publish(msg)
            time.sleep(0.1)
        
        time.sleep(2)  # Wait for movement
        
        print(f"\n📏 Measure the arm position:")
        try:
            x = float(input("  X position (cm): "))
            y = float(input("  Y position (cm): "))
            z = float(input("  Z position (cm): "))
        except ValueError:
            print("Invalid input!")
            return None
        
        return {'angle': angle, 'x': x, 'y': y, 'z': z}
    
    def measure_base_servo(self):
        """Measure base servo limits."""
        print("\n" + "=" * 70)
        print("MEASURING BASE SERVO LIMITS")
        print("=" * 70)
        print("\nThe base servo rotates the entire arm left/right.")
        print("We'll find its actual physical limits.\n")
        
        measurements = []
        
        # Test different base angles
        test_angles = [0, 45, 90, 135, 180]
        
        for angle in test_angles:
            result = self.move_servo(5, angle, f"Base at {angle}°")
            if result:
                measurements.append(result)
                print(f"  → X={result['x']:.1f}, Y={result['y']:.1f}, Z={result['z']:.1f}")
        
        return measurements
    
    def analyze_results(self, measurements):
        """Analyze what we learned."""
        print("\n" + "=" * 70)
        print("ANALYSIS")
        print("=" * 70)
        
        print("\nBase Servo Measurements:")
        for m in measurements:
            print(f"  {m['angle']:3.0f}° → X={m['x']:6.2f}, Y={m['y']:6.2f}, Z={m['z']:6.2f}")
        
        # Find min/max X and Y
        x_values = [m['x'] for m in measurements]
        y_values = [m['y'] for m in measurements]
        
        print(f"\nX range: {min(x_values):.2f} to {max(x_values):.2f} cm")
        print(f"Y range: {min(y_values):.2f} to {max(y_values):.2f} cm")
        
        # Calculate the actual angle range
        # At 90°, radius is 25cm
        # At other angles, radius is smaller
        print("\nKey findings:")
        print("  - At 90°: X=25, Y=0 (full forward reach)")
        print(f"  - At 0°/180°: X={min(x_values):.1f} (limited range!)")
        print("\nThis means the base servo doesn't rotate a full 180°!")
        print("It has a LIMITED physical range.\n")


def main():
    rclpy.init()
    node = ServoLimitsMeasurer()
    
    try:
        measurements = node.measure_base_servo()
        node.analyze_results(measurements)
        
        print("\nNEXT STEPS:")
        print("  1. Note the actual X/Y ranges you measured")
        print("  2. I'll update the FK to match these limits")
        print("  3. The arm will display correct positions!")
        
    except KeyboardInterrupt:
        print("\n\nStopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
