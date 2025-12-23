import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')
        
        i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 60
        
        self.subscription = self.create_subscription(
            Float32,
            'servo_command',
            self.servo_callback,
            10
        )
        
        self.get_logger().info('Servo node started (ROS2 Jazzy)')
        self.get_logger().info('Listening on: /servo_command')
    
    def servo_callback(self, msg):
        angle = max(0, min(180, msg.data))
        pulse = int(0x0CCC + (angle / 180.0) * (0x1999 - 0x0CCC))
        self.pca.channels[0].duty_cycle = pulse
        self.get_logger().info(f'Servo: {angle}°')

def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
