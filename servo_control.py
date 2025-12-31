"""
ROS2 Robotic Arm Control Node
------------------------------
Controls 6 servo motors for a robotic arm using PCA9685 PWM driver.
Includes safe initialization.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio
import time


class RoboticArmNode(Node):
    def __init__(self):
        """Initialize the robotic arm node with safe servo startup."""
        super().__init__('robotic_arm_node')
        
        # Hardware setup
        i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 60
        
        self.NUM_SERVOS = 6
        self.servo_names = ["Gripper", "Wrist Roll", "Wrist Pitch", "Elbow", "Shoulder", "Base"]
        
        # Current state
        self.current_angles = [90.0] * self.NUM_SERVOS
        self.initialized = False
        
        # Movement parameters
        self.movement_speed = 2.0  # degrees per step
        self.step_delay = 0.02     # seconds between steps
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('ROBOTIC ARM INITIALIZATION')
        self.get_logger().info('=' * 60)
        
        # SAFE STARTUP: Disable all PWM first
        self.get_logger().info('Step 1: Disabling all PWM outputs...')
        for channel in range(self.NUM_SERVOS):
            self.pca.channels[channel].duty_cycle = 0
        time.sleep(1.0)
        self.get_logger().info('✓ All servos disabled')
        
        # Create ROS2 subscribers
        self.subscription = self.create_subscription(
            Float32MultiArray, 'arm_command', self.arm_callback, 10)
        self.demo_subscription = self.create_subscription(
            String, 'arm_demo', self.demo_callback, 10)
        
        # Wait for topics to settle
        self.get_logger().info('Step 2: Waiting for ROS2 topics to settle...')
        time.sleep(1.0)
        
        # SAFE STARTUP: Slowly move to center
        self.get_logger().info('Step 3: Moving servos to center (90°)...')
        self.get_logger().info('This takes ~15 seconds - please wait')
        
        for channel in range(self.NUM_SERVOS):
            self.get_logger().info(f'  Initializing {self.servo_names[channel]}...')
            
            # Move in small steps to 90°
            steps = 60
            for step in range(steps + 1):
                self.set_servo_angle(channel, 90.0)
                time.sleep(0.025)
            
            self.current_angles[channel] = 90.0
            self.get_logger().info(f'    ✓ {self.servo_names[channel]} centered')
        
        # Ready!
        self.initialized = True
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('✓ READY!')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Topics: /arm_command, /arm_demo')
        self.get_logger().info(f'Movement: {self.movement_speed}°/step @ {self.step_delay*1000:.0f}ms')
        self.get_logger().info('=' * 60)
    
    def get_pulse_range(self, channel):
        """Get PWM pulse range for servo channel."""
        return 0x0CCC, 0x1999  # Same for all servos
    
    def set_servo_angle(self, channel, angle):
        """Set servo angle (0-180 degrees)."""
        angle = max(0, min(180, angle))
        min_pulse, max_pulse = self.get_pulse_range(channel)
        pulse = int(min_pulse + (angle / 180.0) * (max_pulse - min_pulse))
        self.pca.channels[channel].duty_cycle = pulse
    
    def smooth_move_to_angles(self, target_angles):
        """Smoothly interpolate from current to target angles."""
        max_diff = max(abs(target_angles[i] - self.current_angles[i]) 
                      for i in range(self.NUM_SERVOS))
        
        if max_diff == 0:
            return
        
        num_steps = max(1, int(max_diff / self.movement_speed))
        
        for step in range(num_steps + 1):
            t = step / num_steps
            for channel in range(self.NUM_SERVOS):
                angle = self.current_angles[channel] + t * (
                    target_angles[channel] - self.current_angles[channel])
                self.set_servo_angle(channel, angle)
            
            if step < num_steps:
                time.sleep(self.step_delay)
        
        self.current_angles = list(target_angles)
    
    def arm_callback(self, msg):
        """Direct motor angle control."""
        if not self.initialized:
            self.get_logger().warn('Ignoring command - still initializing')
            return
        
        if len(msg.data) != self.NUM_SERVOS:
            self.get_logger().error(f'Expected {self.NUM_SERVOS} angles')
            return
        
        self.smooth_move_to_angles(list(msg.data))
        
        # Log servo angles
        self.get_logger().info(
            f'Angles: [{msg.data[0]:.1f}° {msg.data[1]:.1f}° {msg.data[2]:.1f}° '
            f'{msg.data[3]:.1f}° {msg.data[4]:.1f}° {msg.data[5]:.1f}°]'
        )
    
    def demo_callback(self, msg):
        """Demo commands."""
        if not self.initialized:
            return
        
        if msg.data == 'center':
            self.get_logger().info('Centering all motors...')
            self.smooth_move_to_angles([90.0] * self.NUM_SERVOS)
            self.get_logger().info('✓ Centered')
    
    def disable_all_servos(self):
        """Disable all servos on shutdown."""
        for channel in range(self.NUM_SERVOS):
            self.pca.channels[channel].duty_cycle = 0
        self.get_logger().info('All servos disabled')


def main(args=None):
    rclpy.init(args=args)
    node = RoboticArmNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.disable_all_servos()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
