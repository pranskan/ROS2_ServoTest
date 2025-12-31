"""
Find actual servo motor limits
Test each servo individually to find min/max angles
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time


class ServoLimitsFinder(Node):
    def __init__(self):
        super().__init__('servo_limits_finder')
        self.arm_pub = self.create_publisher(Float32MultiArray, 'arm_command', 10)
        time.sleep(2)
        self.get_logger().info('Ready to find servo limits')
    
    def test_servo(self, servo_channel, servo_name):
        """Test a single servo to find its limits."""
        print("\n" + "=" * 70)
        print(f"TESTING: {servo_name} (Channel {servo_channel})")
        print("=" * 70)
        
        limits = {'min': 0, 'max': 180}
        
        # Test minimum
        print("\nTesting MINIMUM angle...")
        print("Trying 0°...")
        angles = [90.0] * 6
        angles[servo_channel] = 0
        msg = Float32MultiArray()
        msg.data = angles
        for _ in range(5):
            self.arm_pub.publish(msg)
            time.sleep(0.1)
        time.sleep(1)
        
        response = input("Does servo move to 0°? (y/n): ").lower()
        if response == 'y':
            limits['min'] = 0
            print("✓ Can reach 0°")
        else:
            print("✗ Cannot reach 0° - testing higher angles")
            for test_angle in [20, 30, 40, 45, 50]:
                angles[servo_channel] = test_angle
                msg.data = angles
                for _ in range(5):
                    self.arm_pub.publish(msg)
                    time.sleep(0.1)
                time.sleep(1)
                
                response = input(f"Does servo move to {test_angle}°? (y/n): ").lower()
                if response == 'y':
                    limits['min'] = test_angle
                    print(f"✓ Minimum angle is {test_angle}°")
                    break
        
        # Test maximum
        print("\nTesting MAXIMUM angle...")
        print("Trying 180°...")
        angles[servo_channel] = 180
        msg.data = angles
        for _ in range(5):
            self.arm_pub.publish(msg)
            time.sleep(0.1)
        time.sleep(1)
        
        response = input("Does servo move to 180°? (y/n): ").lower()
        if response == 'y':
            limits['max'] = 180
            print("✓ Can reach 180°")
        else:
            print("✗ Cannot reach 180° - testing lower angles")
            for test_angle in [160, 150, 140, 135, 130]:
                angles[servo_channel] = test_angle
                msg.data = angles
                for _ in range(5):
                    self.arm_pub.publish(msg)
                    time.sleep(0.1)
                time.sleep(1)
                
                response = input(f"Does servo move to {test_angle}°? (y/n): ").lower()
                if response == 'y':
                    limits['max'] = test_angle
                    print(f"✓ Maximum angle is {test_angle}°")
                    break
        
        return limits
    
    def run_calibration(self):
        """Test all servos."""
        servo_names = ["Gripper", "Wrist Roll", "Wrist Pitch", "Elbow", "Shoulder", "Base"]
        all_limits = {}
        
        for channel in range(6):
            limits = self.test_servo(channel, servo_names[channel])
            all_limits[channel] = limits
        
        return all_limits


def main():
    rclpy.init()
    node = ServoLimitsFinder()
    
    try:
        all_limits = node.run_calibration()
        
        print("\n" + "=" * 70)
        print("CALIBRATION RESULTS")
        print("=" * 70)
        print("\nServo Angle Limits:\n")
        
        servo_names = ["Gripper", "Wrist Roll", "Wrist Pitch", "Elbow", "Shoulder", "Base"]
        for channel, limits in all_limits.items():
            min_angle = limits['min']
            max_angle = limits['max']
            range_val = max_angle - min_angle
            print(f"  {channel}: {servo_names[channel]:15s} → {min_angle:3.0f}° to {max_angle:3.0f}° (range: {range_val:3.0f}°)")
        
        print("\n" + "=" * 70)
        print("Update servo_control.py with these limits!")
        print("=" * 70)
        
    except KeyboardInterrupt:
        print("\n\nStopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
