"""
Find the center point for each 270° servo motor
Determines where each motor naturally centers (should be used as 0° reference)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time


class ServoCenter Finder(Node):
    def __init__(self):
        super().__init__('servo_center_finder')
        self.arm_pub = self.create_publisher(Float32MultiArray, 'arm_command', 10)
        time.sleep(2)
        self.get_logger().info('Ready to find servo centers')
    
    def move_servo_and_hold(self, servo_channel, angle):
        """Move servo to angle and hold it there."""
        angles = [90.0] * 6
        angles[servo_channel] = angle
        msg = Float32MultiArray()
        msg.data = angles
        
        for _ in range(10):
            self.arm_pub.publish(msg)
            time.sleep(0.1)
        
        return msg
    
    def test_270_servo(self, servo_channel, servo_name):
        """Test a 270° servo to find its actual center."""
        print("\n" + "=" * 70)
        print(f"FINDING CENTER: {servo_name} (Channel {servo_channel})")
        print("=" * 70)
        
        # Test minimum
        print("\nStep 1: Finding MINIMUM angle")
        print("Testing 0°...")
        msg = self.move_servo_and_hold(servo_channel, 0)
        
        for _ in range(5):
            self.arm_pub.publish(msg)
            time.sleep(0.1)
        
        time.sleep(1)
        min_angle = float(input("Does servo move to 0°? If yes, enter the angle it reaches. If no, enter 'n': "))
        
        if min_angle == 'n':
            # Try higher
            for test_angle in [15, 30, 45, 60]:
                msg = self.move_servo_and_hold(servo_channel, test_angle)
                for _ in range(5):
                    self.arm_pub.publish(msg)
                    time.sleep(0.1)
                time.sleep(1)
                
                response = input(f"Does servo move to {test_angle}°? (y/n): ").lower()
                if response == 'y':
                    min_angle = test_angle
                    print(f"✓ Minimum: {min_angle}°")
                    break
        else:
            print(f"✓ Minimum: {min_angle}°")
        
        # Test maximum
        print("\nStep 2: Finding MAXIMUM angle")
        print("Testing 270°...")
        msg = self.move_servo_and_hold(servo_channel, 270)
        
        for _ in range(5):
            self.arm_pub.publish(msg)
            time.sleep(0.1)
        
        time.sleep(1)
        max_angle = float(input("Does servo move to 270°? If yes, enter the angle it reaches. If no, enter 'n': "))
        
        if max_angle == 'n':
            # Try lower
            for test_angle in [255, 240, 225, 210]:
                msg = self.move_servo_and_hold(servo_channel, test_angle)
                for _ in range(5):
                    self.arm_pub.publish(msg)
                    time.sleep(0.1)
                time.sleep(1)
                
                response = input(f"Does servo move to {test_angle}°? (y/n): ").lower()
                if response == 'y':
                    max_angle = test_angle
                    print(f"✓ Maximum: {max_angle}°")
                    break
        else:
            print(f"✓ Maximum: {max_angle}°")
        
        # Calculate center
        center = (min_angle + max_angle) / 2
        
        return {
            'min': min_angle,
            'max': max_angle,
            'center': center
        }
    
    def run_calibration(self):
        """Test all 270° servos."""
        servo_270_motors = [
            (2, "Wrist Pitch"),
            (3, "Elbow"),
            (4, "Shoulder"),
            (5, "Base"),
        ]
        
        results = {}
        
        for channel, name in servo_270_motors:
            result = self.test_270_servo(channel, name)
            results[channel] = result
        
        return results


def main():
    rclpy.init()
    node = ServoCenter Finder()
    
    try:
        results = node.run_calibration()
        
        print("\n" + "=" * 70)
        print("CENTER CALIBRATION RESULTS")
        print("=" * 70)
        print("\nServo Centers for 270° Motors:\n")
        
        servo_names = {
            2: "Wrist Pitch",
            3: "Elbow",
            4: "Shoulder",
            5: "Base",
        }
        
        for channel, data in results.items():
            print(f"{servo_names[channel]}:")
            print(f"  Min: {data['min']:.1f}°")
            print(f"  Max: {data['max']:.1f}°")
            print(f"  Center: {data['center']:.1f}°")
            print()
        
        print("=" * 70)
        print("UPDATE kinematics_dh.py with these center values!")
        print("Replace 135.0 with the actual center for each motor")
        print("=" * 70)
        
    except KeyboardInterrupt:
        print("\n\nStopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
