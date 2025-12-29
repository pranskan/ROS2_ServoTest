"""
ROS2 Robotic Arm Control Node
-----------------------------
Controls 6 servo motors for a robotic arm using PCA9685 PWM driver.

Hardware:
- PCA9685 PWM Driver on I2C bus
- 6 servos connected to channels 0-5
- Channels 0-1: MG996R
- Channels 2-5: DS3218
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Point  # Add this import
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio
import time
from kinematics import ArmKinematics  # Add this import


class RoboticArmNode(Node):
    def __init__(self):
        """Initialize the robotic arm node and setup I2C communication."""
        super().__init__('robotic_arm_node')
        
        i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 60
        
        self.NUM_SERVOS = 6
        self.servo_names = ["Gripper", "Wrist Roll", "Wrist Pitch", "Elbow", "Shoulder", "Base"]
        
        # Initialize kinematics solver
        self.arm_kinematics = ArmKinematics()
        
        # Create subscriber for arm commands
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'arm_command',
            self.arm_callback,
            10
        )
        
        # Create subscriber for demo/test commands
        self.demo_subscription = self.create_subscription(
            String,
            'arm_demo',
            self.demo_callback,
            10
        )
        
        # Create subscriber for XYZ commands
        self.xyz_subscription = self.create_subscription(
            Point,
            'arm_xyz',
            self.xyz_callback,
            10
        )
        
        # Initialize all servos to center position
        self.get_logger().info('Initializing robotic arm...')
        for channel in range(self.NUM_SERVOS):
            self.set_servo_angle(channel, 90.0)
            time.sleep(0.2)  # Slower initialization to prevent jerky movements
        
        self.get_logger().info('Robotic Arm node started (ROS2 Jazzy)')
        self.get_logger().info('Listening on: /arm_command, /arm_demo, /arm_xyz')
        self.get_logger().info(f'Controlling {self.NUM_SERVOS} servos on channels 0-{self.NUM_SERVOS-1}')
        self.get_logger().info('Servo types: Ch0-1=MG996R, Ch2-5=DS3218')
        self.get_logger().info('All servos initialized to 90° (center)')
    
    def get_pulse_range(self, channel):
        """Get the pulse range for a given servo channel."""
        if channel in [0, 1]:  # MG996R
            return 0x0CCC, 0x1999
        elif channel in [2, 3, 4, 5]:  # DS3218
            return 0x0CCC, 0x1999
        else:
            raise ValueError(f"Invalid channel: {channel}")
    
    def set_servo_angle(self, channel, angle):
        """Set the angle of a servo motor."""
        angle = max(0, min(180, angle))
        min_pulse, max_pulse = self.get_pulse_range(channel)
        pulse = int(min_pulse + (angle / 180.0) * (max_pulse - min_pulse))
        self.pca.channels[channel].duty_cycle = pulse
        self.get_logger().info(f'{self.servo_names[channel]} (Ch{channel}): {angle}°')
    
    def run_demo(self):
        """Run through preset test positions to demonstrate arm movements."""
        self.get_logger().info('Starting demo sequence...')
        
        # Define demo positions
        # Format: ([ch0, ch1, ch2, ch3, ch4, ch5], "description", pause_time)
        demo_positions = [
            # Start position
            ([90, 90, 90, 90, 90, 90], "HOME: All servos centered", 2),
            
            # Test base rotation (channel 5)
            ([90, 90, 90, 90, 90, 45], "BASE: Rotate right", 2),
            ([90, 90, 90, 90, 90, 135], "BASE: Rotate left", 2),
            ([90, 90, 90, 90, 90, 90], "BASE: Return center", 2),
            
            # Test shoulder (channel 4)
            ([90, 90, 90, 90, 45, 90], "SHOULDER: Lift up", 2),
            ([90, 90, 90, 90, 135, 90], "SHOULDER: Lower down", 2),
            ([90, 90, 90, 90, 90, 90], "SHOULDER: Return center", 2),
            
            # Test elbow (channel 3)
            ([90, 90, 90, 45, 90, 90], "ELBOW: Extend", 2),
            ([90, 90, 90, 135, 90, 90], "ELBOW: Retract", 2),
            ([90, 90, 90, 90, 90, 90], "ELBOW: Return center", 2),
            
            # Test wrist pitch (channel 2)
            ([90, 90, 45, 90, 90, 90], "WRIST PITCH: Tilt up", 2),
            ([90, 90, 135, 90, 90, 90], "WRIST PITCH: Tilt down", 2),
            ([90, 90, 90, 90, 90, 90], "WRIST PITCH: Return center", 2),
            
            # Test wrist roll (channel 1)
            ([90, 45, 90, 90, 90, 90], "WRIST ROLL: Rotate CW", 2),
            ([90, 135, 90, 90, 90, 90], "WRIST ROLL: Rotate CCW", 2),
            ([90, 90, 90, 90, 90, 90], "WRIST ROLL: Return center", 2),
            
            # Test gripper (channel 0)
            ([0, 90, 90, 90, 90, 90], "GRIPPER: Close", 2),
            ([180, 90, 90, 90, 90, 90], "GRIPPER: Open", 2),
            ([90, 90, 90, 90, 90, 90], "GRIPPER: Half open", 2),
            
            # Combined movements
            ([90, 90, 90, 45, 45, 90], "COMBO: Reach forward", 2),
            ([90, 90, 90, 135, 135, 90], "COMBO: Reach up", 2),
            ([0, 90, 90, 90, 90, 135], "COMBO: Grab object left", 3),
            ([180, 90, 90, 90, 90, 45], "COMBO: Release object right", 3),
            
            # Return home
            ([90, 90, 90, 90, 90, 90], "HOME: Return to center", 2),
        ]
        
        for position_num, (angles, description, pause) in enumerate(demo_positions, 1):
            self.get_logger().info(f'Position {position_num}/{len(demo_positions)}: {description}')
            
            # Move each servo
            for channel, angle in enumerate(angles):
                self.set_servo_angle(channel, angle)
            
            # Pause
            time.sleep(pause)
        
        self.get_logger().info('Demo complete!')
    
    def sweep_motor(self, channel):
        """Sweep a single motor through its full range."""
        if channel < 0 or channel >= self.NUM_SERVOS:
            self.get_logger().error(f'Invalid channel {channel}')
            return
        
        self.get_logger().info(f'Sweeping {self.servo_names[channel]} (Channel {channel})')
        
        sweep_angles = [0, 90, 180, 90, 0, 90]
        
        for angle in sweep_angles:
            self.get_logger().info(f'  → {angle}°')
            self.set_servo_angle(channel, angle)
            time.sleep(1.5)
        
        self.get_logger().info('Sweep complete')
    
    def demo_callback(self, msg):
        """
        Callback for demo commands.
        
        Args:
            msg (String): Command string
                "demo" - Run full demo sequence
                "sweep:0" - Sweep channel 0
                "sweep:5" - Sweep channel 5
                "center" - Return all servos to 90°
        """
        command = msg.data.lower().strip()
        
        if command == 'demo':
            self.run_demo()
        elif command.startswith('sweep:'):
            try:
                channel = int(command.split(':')[1])
                self.sweep_motor(channel)
            except (ValueError, IndexError):
                self.get_logger().error('Invalid sweep command. Use: sweep:0 to sweep:5')
        elif command == 'center':
            self.get_logger().info('Moving all servos to center (90°)...')
            for channel in range(self.NUM_SERVOS):
                self.set_servo_angle(channel, 90.0)
                time.sleep(0.2)
            self.get_logger().info('All servos centered')
        else:
            self.get_logger().error(f'Unknown demo command: {command}')
    
    def arm_callback(self, msg):
        """Callback for arm commands."""
        if len(msg.data) != self.NUM_SERVOS:
            self.get_logger().error(f'Invalid command length: {len(msg.data)}. Expected {self.NUM_SERVOS}.')
            return
        
        for channel, angle in enumerate(msg.data):
            self.set_servo_angle(channel, angle)
    
    def xyz_callback(self, msg):
        """
        Callback for XYZ position commands.
        
        Args:
            msg (Point): Target position
                msg.x: X coordinate (cm)
                msg.y: Y coordinate (cm)
                msg.z: Z coordinate (cm)
        """
        x, y, z = msg.x, msg.y, msg.z
        
        self.get_logger().info(f'Moving to position: X={x:.2f}, Y={y:.2f}, Z={z:.2f} cm')
        
        # Calculate joint angles using inverse kinematics
        logical_angles = self.arm_kinematics.inverse_kinematics(x, y, z)
        
        if logical_angles is None:
            self.get_logger().error(f'Position ({x}, {y}, {z}) is unreachable')
            return
        
        # Map logical angles to physical channels
        # logical_angles = [base, shoulder, elbow, wrist_pitch, wrist_roll, gripper]
        # channels       = [5,    4,        3,     2,           1,          0]
        channel_mapping = {
            5: logical_angles[0],  # Base -> Channel 5
            4: logical_angles[1],  # Shoulder -> Channel 4
            3: logical_angles[2],  # Elbow -> Channel 3
            2: logical_angles[3],  # Wrist Pitch -> Channel 2
            1: logical_angles[4],  # Wrist Roll -> Channel 1
            0: logical_angles[5],  # Gripper -> Channel 0
        }
        
        # Move servos to calculated positions
        self.get_logger().info('Solution found! Moving to position...')
        for channel, angle in channel_mapping.items():
            self.set_servo_angle(channel, angle)
        
        self.get_logger().info('Position reached!')
    
    def disable_all_servos(self):
        """Disable all servos by setting their duty cycle to 0."""
        for channel in range(self.NUM_SERVOS):
            self.pca.channels[channel].duty_cycle = 0
        self.get_logger().info('All servos disabled')


def main(args=None):
    rclpy.init(args=args)
    node = RoboticArmNode()
    rclpy.spin(node)
    node.disable_all_servos()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
