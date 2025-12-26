"""
ROS2 Robotic Arm Control Node
-----------------------------
Controls 6 servo motors for a robotic arm using PCA9685 PWM driver.

Hardware:
- PCA9685 PWM Driver on I2C bus
- 6 servos connected to channels 0-5
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio


class RoboticArmNode(Node):
    """
    ROS2 Node that controls a 6-DOF robotic arm via PCA9685 PWM driver.
    
    Subscribes to: /arm_command (Float32MultiArray) - Array of 6 angles (0-180°)
    
    ACTUAL WIRING - Channel to Joint mapping:
    - Channel 0: Gripper
    - Channel 1: Wrist Roll
    - Channel 2: Wrist Pitch
    - Channel 3: Elbow
    - Channel 4: Shoulder
    - Channel 5: Base rotation
    
    Message format (logical order):
    /arm_command [base, shoulder, elbow, wrist_pitch, wrist_roll, gripper]
    """
    
    def __init__(self):
        """Initialize the robotic arm node and setup I2C communication."""
        super().__init__('robotic_arm_node')
        
        # Setup I2C bus
        i2c = busio.I2C(SCL, SDA)
        
        # Initialize PCA9685 PWM driver
        self.pca = PCA9685(i2c)
        self.pca.frequency = 60
        
        # PWM pulse range for standard servos
        self.SERVO_MIN = 0x0CCC  # Min pulse (0°) - ~0.5ms
        self.SERVO_MAX = 0x1999  # Max pulse (180°) - ~2.5ms
        
        # Number of servos
        self.NUM_SERVOS = 6
        
        # Current positions (for safety checking)
        self.current_angles = [90.0] * self.NUM_SERVOS  # Start at center
        
        # Servo names by channel (actual wiring)
        self.servo_names = [
            "Gripper",       # Channel 0
            "Wrist Roll",    # Channel 1
            "Wrist Pitch",   # Channel 2
            "Elbow",         # Channel 3
            "Shoulder",      # Channel 4
            "Base",          # Channel 5
        ]
        
        # Logical joint names (for messages)
        self.logical_joint_names = [
            "Base",          # Logical index 0 -> Channel 5
            "Shoulder",      # Logical index 1 -> Channel 4
            "Elbow",         # Logical index 2 -> Channel 3
            "Wrist Pitch",   # Logical index 3 -> Channel 2
            "Wrist Roll",    # Logical index 4 -> Channel 1
            "Gripper",       # Logical index 5 -> Channel 0
        ]
        
        # Create subscriber for arm commands
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'arm_command',
            self.arm_callback,
            10
        )
        
        # Initialize all servos to center position
        self.get_logger().info('Initializing robotic arm...')
        for channel in range(self.NUM_SERVOS):
            self.set_servo_angle(channel, 90.0)
        
        self.get_logger().info('Robotic Arm node started (ROS2 Jazzy)')
        self.get_logger().info('Listening on: /arm_command')
        self.get_logger().info(f'Controlling {self.NUM_SERVOS} servos on channels 0-{self.NUM_SERVOS-1}')
        self.get_logger().info('All servos initialized to 90° (center)')
    
    def set_servo_angle(self, channel, angle):
        """
        Set servo angle on a specific channel.
        
        Args:
            channel (int): PCA9685 channel (0-5)
            angle (float): Target angle (0-180 degrees)
        """
        # Clamp angle to safe range
        angle = max(0, min(180, angle))
        
        # Convert angle to PWM duty cycle
        pulse = int(self.SERVO_MIN + (angle / 180.0) * (self.SERVO_MAX - self.SERVO_MIN))
        
        # Send PWM signal
        self.pca.channels[channel].duty_cycle = pulse
        
        # Update current position
        self.current_angles[channel] = angle
    
    def disable_all_servos(self):
        """Disable all PWM outputs to stop current draw."""
        self.get_logger().info('Disabling all PWM outputs...')
        for channel in range(self.NUM_SERVOS):
            self.pca.channels[channel].duty_cycle = 0
        self.get_logger().info('All PWM outputs disabled')
    
    def arm_callback(self, msg):
        """
        Callback for arm command messages.
        
        Args:
            msg (Float32MultiArray): Array of 6 angles (0-180°)
                Logical order: [base, shoulder, elbow, wrist_pitch, wrist_roll, gripper]
                
        Channel Mapping:
            msg.data[0] (base)        -> Channel 5
            msg.data[1] (shoulder)    -> Channel 4
            msg.data[2] (elbow)       -> Channel 3
            msg.data[3] (wrist_pitch) -> Channel 2
            msg.data[4] (wrist_roll)  -> Channel 1
            msg.data[5] (gripper)     -> Channel 0
        """
        # Validate message has correct number of values
        if len(msg.data) != self.NUM_SERVOS:
            self.get_logger().error(
                f'Expected {self.NUM_SERVOS} angles, got {len(msg.data)}'
            )
            return
        
        # Map logical joint order to physical channels
        channel_mapping = {
            5: msg.data[0],  # Base -> Channel 5
            4: msg.data[1],  # Shoulder -> Channel 4
            3: msg.data[2],  # Elbow -> Channel 3
            2: msg.data[3],  # Wrist Pitch -> Channel 2
            1: msg.data[4],  # Wrist Roll -> Channel 1
            0: msg.data[5],  # Gripper -> Channel 0
        }
        
        # Move each servo to target angle
        log_msg = "Moving: "
        for logical_idx, angle in enumerate(msg.data):
            # Find which channel this logical joint maps to
            if logical_idx == 0:   # Base
                channel = 5
            elif logical_idx == 1: # Shoulder
                channel = 4
            elif logical_idx == 2: # Elbow
                channel = 3
            elif logical_idx == 3: # Wrist Pitch
                channel = 2
            elif logical_idx == 4: # Wrist Roll
                channel = 1
            elif logical_idx == 5: # Gripper
                channel = 0
            
            self.set_servo_angle(channel, angle)
            log_msg += f"{self.logical_joint_names[logical_idx]}={angle:.1f}° "
        
        self.get_logger().info(log_msg)


def main(args=None):
    """Main entry point for the robotic arm node."""
    rclpy.init(args=args)
    node = RoboticArmNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down robotic arm...')
    finally:
        # Disable all PWM outputs before shutdown
        node.disable_all_servos()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
