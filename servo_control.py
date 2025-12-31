"""
ROS2 Robotic Arm Control Node
------------------------------
Controls 6 servo motors for a robotic arm using PCA9685 PWM driver.
Includes safe initialization and forward kinematics.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio
import time
from kinematics_dh import ArmKinematicsDH


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
        
        # Initialize kinematics with measured dimensions
        self.arm_kinematics = ArmKinematicsDH(
            base_height=10.5,
            l2=12.9,
            l3=11.0,
            l4=15.0
        )
        
        # Servo angle limits (min, max) - UPDATE THESE BASED ON CALIBRATION
        # Default: full range, but adjust per servo based on physical limits
        self.servo_limits = [
            (0, 180),    # 0: Gripper
            (0, 180),    # 1: Wrist Roll
            (45, 135),   # 2: Wrist Pitch - LIKELY LIMITED (~90° range)
            (45, 135),   # 3: Elbow - LIKELY LIMITED (~90° range)
            (45, 135),   # 4: Shoulder - LIKELY LIMITED (~90° range)
            (45, 135),   # 5: Base - LIKELY LIMITED (~90° range)
        ]
        
        # Current state
        self.current_angles = [90.0] * self.NUM_SERVOS
        self.current_xyz = self.get_xyz_position(self.current_angles)
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
        
        # Calculate initial position
        self.current_xyz = self.get_xyz_position(self.current_angles)
        
        # Ready!
        self.initialized = True
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('✓ READY!')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Topics: /arm_command, /arm_demo')
        self.get_logger().info(f'Movement: {self.movement_speed}°/step @ {self.step_delay*1000:.0f}ms')
        self.get_logger().info(f'Position: X={self.current_xyz["x"]:.2f}, Y={self.current_xyz["y"]:.2f}, Z={self.current_xyz["z"]:.2f} cm')
        self.get_logger().info('=' * 60)
    
    def get_pulse_range(self, channel):
        """Get PWM pulse range for servo channel."""
        return 0x0CCC, 0x1999  # Same for all servos
    
    def set_servo_angle(self, channel, angle):
        """Set servo angle with per-servo limits."""
        min_limit, max_limit = self.servo_limits[channel]
        angle = max(min_limit, min(max_limit, angle))
        min_pulse, max_pulse = self.get_pulse_range(channel)
        pulse = int(min_pulse + (angle / 180.0) * (max_pulse - min_pulse))
        self.pca.channels[channel].duty_cycle = pulse
    
    def get_xyz_position(self, angles):
        """Calculate XYZ position from joint angles using forward kinematics."""
        pos = self.arm_kinematics.forward_kinematics(angles)
        return pos
    
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
        self.current_xyz = self.get_xyz_position(self.current_angles)
    
    def arm_callback(self, msg):
        """Direct motor angle control."""
        if not self.initialized:
            self.get_logger().warn('Ignoring command - still initializing')
            return
        
        if len(msg.data) != self.NUM_SERVOS:
            self.get_logger().error(f'Expected {self.NUM_SERVOS} angles')
            return
        
        self.smooth_move_to_angles(list(msg.data))
        
        # Log servo angles AND XYZ position
        self.get_logger().info(
            f'Angles: [{msg.data[0]:.1f}° {msg.data[1]:.1f}° {msg.data[2]:.1f}° '
            f'{msg.data[3]:.1f}° {msg.data[4]:.1f}° {msg.data[5]:.1f}°] | '
            f'XYZ: ({self.current_xyz["x"]:.2f}, {self.current_xyz["y"]:.2f}, {self.current_xyz["z"]:.2f}) cm'
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
