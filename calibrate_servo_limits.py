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
    
    def move_servo_and_hold(self, servo_channel, angle):
        """Move servo to angle and hold it there."""
        angles = [90.0] * 6
        angles[servo_channel] = angle
        msg = Float32MultiArray()
        msg.data = angles
        
        # Publish multiple times to ensure it gets there
        for _ in range(10):
            self.arm_pub.publish(msg)
            time.sleep(0.1)
        
        # Keep publishing to hold position while user responds
        return angles, msg
    
    def test_servo(self, servo_channel, servo_name):
        """Test a single servo to find its limits."""
        print("\n" + "=" * 70)
        print(f"TESTING: {servo_name} (Channel {servo_channel})")
        print("=" * 70)
        
        limits = {'min': 0, 'max': 180}
        
        # Test minimum
        print("\nTesting MINIMUM angle...")
        print("Trying 0°...")
        angles, msg = self.move_servo_and_hold(servo_channel, 0)
        
        # Keep holding position while waiting for response
        response = None
        while response is None:
            response_str = input("Does servo move to 0°? (y/n): ").lower()
            if response_str == 'y':
                response = True
                limits['min'] = 0
                print("✓ Can reach 0°")
            elif response_str == 'n':
                response = False
                print("✗ Cannot reach 0° - testing higher angles")
            else:
                print("Please enter 'y' or 'n'")
                response = None
            
            # Keep publishing to hold position
            for _ in range(5):
                self.arm_pub.publish(msg)
                time.sleep(0.1)
        
        # Test higher angles if 0° didn't work
        if not limits['min'] == 0:
            for test_angle in [20, 30, 40, 45, 50]:
                angles, msg = self.move_servo_and_hold(servo_channel, test_angle)
                
                response = None
                while response is None:
                    response_str = input(f"Does servo move to {test_angle}°? (y/n): ").lower()
                    if response_str == 'y':
                        response = True
                        limits['min'] = test_angle
                        print(f"✓ Minimum angle is {test_angle}°")
                    elif response_str == 'n':
                        response = False
                    else:
                        print("Please enter 'y' or 'n'")
                        response = None
                    
                    # Keep publishing to hold position
                    for _ in range(5):
                        self.arm_pub.publish(msg)
                        time.sleep(0.1)
                
                if limits['min'] != 0:
                    break
        
        # Test maximum
        print("\nTesting MAXIMUM angle...")
        print("Trying 180°...")
        angles, msg = self.move_servo_and_hold(servo_channel, 180)
        
        response = None
        while response is None:
            response_str = input("Does servo move to 180°? (y/n): ").lower()
            if response_str == 'y':
                response = True
                limits['max'] = 180
                print("✓ Can reach 180°")
            elif response_str == 'n':
                response = False
                print("✗ Cannot reach 180° - testing lower angles")
            else:
                print("Please enter 'y' or 'n'")
                response = None
            
            # Keep publishing to hold position
            for _ in range(5):
                self.arm_pub.publish(msg)
                time.sleep(0.1)
        
        # Test lower angles if 180° didn't work
        if not limits['max'] == 180:
            for test_angle in [160, 150, 140, 135, 130]:
                angles, msg = self.move_servo_and_hold(servo_channel, test_angle)
                
                response = None
                while response is None:
                    response_str = input(f"Does servo move to {test_angle}°? (y/n): ").lower()
                    if response_str == 'y':
                        response = True
                        limits['max'] = test_angle
                        print(f"✓ Maximum angle is {test_angle}°")
                    elif response_str == 'n':
                        response = False
                    else:
                        print("Please enter 'y' or 'n'")
                        response = None
                    
                    # Keep publishing to hold position
                    for _ in range(5):
                        self.arm_pub.publish(msg)
                        time.sleep(0.1)
                
                if limits['max'] != 180:
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
