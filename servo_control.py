"""
ROS2 Servo Control Node
-----------------------
This node subscribes to the /servo_command topic and moves a servo
connected to a PCA9685 PWM driver board based on the received angle.

Hardware:
- PCA9685 PWM Driver on I2C bus
- Servo connected to channel 0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio


class ServoNode(Node):
    """
    ROS2 Node that controls a servo motor via PCA9685 PWM driver.
    
    Subscribes to: /servo_command (Float32) - Target angle in degrees (0-180)
    """
    
    def __init__(self):
        """Initialize the servo node and setup I2C communication."""
        # Initialize the ROS2 node with name 'servo_node'
        super().__init__('servo_node')
        
        # Setup I2C bus using Raspberry Pi's hardware I2C pins
        # SCL = GPIO 3 (Pin 5), SDA = GPIO 2 (Pin 3)
        i2c = busio.I2C(SCL, SDA)
        
        # Initialize PCA9685 PWM driver on the I2C bus
        self.pca = PCA9685(i2c)
        
        # Set PWM frequency to 60Hz (standard for servos)
        # Most servos expect 50-60Hz signal
        self.pca.frequency = 60
        
        # Create a subscriber that listens for servo commands
        # - Message type: Float32 (angle in degrees)
        # - Topic name: /servo_command
        # - Callback: servo_callback (called when message received)
        # - Queue size: 10 (buffer last 10 messages)
        self.subscription = self.create_subscription(
            Float32,
            'servo_command',
            self.servo_callback,
            10
        )
        
        # Log startup messages
        self.get_logger().info('Servo node started (ROS2 Jazzy)')
        self.get_logger().info('Listening on: /servo_command')
    
    def servo_callback(self, msg):
        """
        Callback function triggered when a message is received on /servo_command.
        
        Args:
            msg (Float32): Message containing target angle (0-180 degrees)
        """
        # Clamp angle between 0 and 180 degrees to prevent servo damage
        # max(0, ...) ensures minimum is 0
        # min(180, ...) ensures maximum is 180 
        angle = max(0, min(180, msg.data))
        
        # Convert angle to PWM duty cycle value
        # PCA9685 uses 12-bit resolution (0-4095)
        # 0x0CCC (3276) ≈ 0.5ms pulse (0°)
        # 0x1999 (6553) ≈ 2.5ms pulse (180°)
        # Linear interpolation: duty = min + (angle/180) * (max - min)
        pulse = int(0x0CCC + (angle / 180.0) * (0x1999 - 0x0CCC))
        
        # Send PWM signal to servo on channel 0
        self.pca.channels[0].duty_cycle = pulse
        
        # Log the angle we just moved to
        self.get_logger().info(f'Servo: {angle}°')


def main(args=None):
    """
    Main entry point for the ROS2 node.
    
    This function:
    1. Initializes ROS2
    2. Creates the servo node
    3. Keeps the node running (spin)
    4. Cleans up on shutdown
    """
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of our ServoNode
    node = ServoNode()
    
    # Keep the node running and processing callbacks
    # This blocks until Ctrl+C or rclpy.shutdown() is called
    rclpy.spin(node)
    
    # Clean up: destroy the node object
    node.destroy_node()
    
    # Shutdown ROS2 Python client library
    rclpy.shutdown()


# Standard Python idiom to run main() when script is executed
if __name__ == '__main__':
    main()
