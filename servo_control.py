"""
ROS2 Robotic Arm Control Node with Path Planning
------------------------------------------------
Controls 6 servo motors for a robotic arm using PCA9685 PWM driver.
Includes path planning for obstacle avoidance.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Point
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio
import time
from kinematics import ArmKinematics
from path_planner import PathPlanner  # Add path planner


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
        
        # Initialize path planner with kinematics
        self.path_planner = PathPlanner(self.arm_kinematics)
        
        # Track current XYZ position (start at center position)
        self.current_xyz = self.get_current_xyz_position([90.0] * 6)
        
        # Track current motor angles
        self.current_angles = [90.0] * self.NUM_SERVOS
        
        # Path planning enabled by default
        self.use_path_planning = True
        
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
        
        # Create subscriber for obstacle management
        self.obstacle_subscription = self.create_subscription(
            String,
            'arm_obstacles',
            self.obstacle_callback,
            10
        )
        
        # Initialize all servos to center position
        self.get_logger().info('Initializing robotic arm...')
        for channel in range(self.NUM_SERVOS):
            self.set_servo_angle(channel, 90.0)
            time.sleep(0.2)  # Slower initialization to prevent jerky movements
        
        self.get_logger().info('Robotic Arm node started (ROS2 Jazzy)')
        self.get_logger().info('Listening on: /arm_command, /arm_demo, /arm_xyz, /arm_obstacles')
        self.get_logger().info(f'Controlling {self.NUM_SERVOS} servos on channels 0-{self.NUM_SERVOS-1}')
        self.get_logger().info('Servo types: Ch0-1=MG996R, Ch2-5=DS3218')
        self.get_logger().info('Path planning: ENABLED')
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
    
    def get_current_xyz_position(self, angles):
        """Calculate current XYZ position from motor angles."""
        # Convert motor channel order to logical joint order
        logical_angles = [
            angles[5],  # Base
            angles[4],  # Shoulder
            angles[3],  # Elbow
            angles[2],  # Wrist Pitch
            angles[1],  # Wrist Roll
            angles[0],  # Gripper
        ]
        position = self.arm_kinematics.forward_kinematics(logical_angles)
        return (position['x'], position['y'], position['z'])
    
    def move_to_waypoint(self, x, y, z):
        """Move to a single XYZ waypoint."""
        # Calculate joint angles
        logical_angles = self.arm_kinematics.inverse_kinematics(x, y, z)
        
        if logical_angles is None:
            return False
        
        # Map to physical channels
        channel_mapping = {
            5: logical_angles[0],  # Base -> Channel 5
            4: logical_angles[1],  # Shoulder -> Channel 4
            3: logical_angles[2],  # Elbow -> Channel 3
            2: logical_angles[3],  # Wrist Pitch -> Channel 2
            1: logical_angles[4],  # Wrist Roll -> Channel 1
            0: logical_angles[5],  # Gripper -> Channel 0
        }
        
        # Move servos
        for channel, angle in channel_mapping.items():
            self.set_servo_angle(channel, angle)
        
        return True
    
    def run_demo(self):
        """Run through preset test positions."""
        # Demo implementation from previous version
        pass
    
    def sweep_motor(self, channel):
        """Sweep a motor through its range."""
        # Sweep implementation from previous version
        pass
    
    def demo_callback(self, msg):
        """Callback for demo commands."""
        command = msg.data.lower().strip()
        
        if command == 'demo':
            self.run_demo()
        elif command.startswith('sweep:'):
            try:
                channel = int(command.split(':')[1])
                self.sweep_motor(channel)
            except (ValueError, IndexError):
                self.get_logger().error('Invalid sweep command')
        elif command == 'center':
            self.get_logger().info('Centering all motors...')
            for channel in range(self.NUM_SERVOS):
                self.set_servo_angle(channel, 90.0)
                time.sleep(0.2)
            self.get_logger().info('All motors centered')
        else:
            self.get_logger().error(f'Unknown command: {command}')
    
    def arm_callback(self, msg):
        """Callback for direct arm angle commands from teleop."""
        # Add logging FIRST to verify callback is being called
        self.get_logger().info('=== ARM CALLBACK TRIGGERED ===')
        
        if len(msg.data) != self.NUM_SERVOS:
            self.get_logger().error(f'Expected {self.NUM_SERVOS} angles, got {len(msg.data)}')
            return
        
        self.get_logger().info(f'Received arm command: {[f"{a:.1f}°" for a in msg.data]}')
        
        # Move each motor to commanded angle
        for channel, angle in enumerate(msg.data):
            self.get_logger().info(f'Setting channel {channel} to {angle:.1f}°')
            self.current_angles[channel] = angle
            self.set_servo_angle(channel, angle)
        
        # Update current XYZ position
        self.current_xyz = self.get_current_xyz_position(list(msg.data))
        
        # Log movement with XYZ
        self.get_logger().info(f'Moved to XYZ: ({self.current_xyz[0]:.2f}, {self.current_xyz[1]:.2f}, {self.current_xyz[2]:.2f}) cm')
    
    def obstacle_callback(self, msg):
        """
        Callback for obstacle management commands.
        
        Commands:
            "add:table:-50,50,-50,50,0,5"  - Add obstacle
            "remove:table"                  - Remove obstacle
            "list"                          - List all obstacles
            "clear"                         - Clear all obstacles
            "enable_planning"               - Enable path planning
            "disable_planning"              - Disable path planning
        """
        command = msg.data.strip()
        
        if command.startswith('add:'):
            # Format: "add:name:x_min,x_max,y_min,y_max,z_min,z_max"
            parts = command.split(':')
            if len(parts) == 3:
                name = parts[1]
                bounds = tuple(map(float, parts[2].split(',')))
                if len(bounds) == 6:
                    self.path_planner.add_obstacle(name, bounds)
                    self.get_logger().info(f'Added obstacle: {name}')
                else:
                    self.get_logger().error('Invalid obstacle bounds format')
            else:
                self.get_logger().error('Invalid add command format')
        
        elif command.startswith('remove:'):
            name = command.split(':')[1]
            self.path_planner.remove_obstacle(name)
            self.get_logger().info(f'Removed obstacle: {name}')
        
        elif command == 'list':
            self.get_logger().info(f'Current obstacles: {list(self.path_planner.obstacles.keys())}')
        
        elif command == 'clear':
            self.path_planner.obstacles = {}
            self.get_logger().info('Cleared all obstacles')
        
        elif command == 'enable_planning':
            self.use_path_planning = True
            self.get_logger().info('Path planning ENABLED')
        
        elif command == 'disable_planning':
            self.use_path_planning = False
            self.get_logger().info('Path planning DISABLED')
        
        else:
            self.get_logger().error(f'Unknown obstacle command: {command}')
    
    def xyz_callback(self, msg):
        """
        Callback for XYZ position commands with path planning.
        """
        target_xyz = (msg.x, msg.y, msg.z)
        
        self.get_logger().info(f'Moving from ({self.current_xyz[0]:.2f}, {self.current_xyz[1]:.2f}, {self.current_xyz[2]:.2f})')
        self.get_logger().info(f'        to   ({target_xyz[0]:.2f}, {target_xyz[1]:.2f}, {target_xyz[2]:.2f}) cm')
        
        if self.use_path_planning:
            # Use path planning
            self.get_logger().info('Using path planning...')
            path = self.path_planner.plan_best_path(self.current_xyz, target_xyz)
            
            if path is None:
                self.get_logger().error('No safe path found!')
                return
            
            # Execute path
            self.get_logger().info(f'Executing path with {len(path)} waypoints...')
            for i, waypoint in enumerate(path):
                success = self.move_to_waypoint(*waypoint)
                if not success:
                    self.get_logger().error(f'Failed at waypoint {i}')
                    return
                time.sleep(0.1)  # Small delay between waypoints
            
            self.get_logger().info('Path execution complete!')
        
        else:
            # Direct movement (no path planning)
            self.get_logger().info('Direct movement (no path planning)...')
            success = self.move_to_waypoint(*target_xyz)
            if not success:
                self.get_logger().error(f'Position ({target_xyz[0]}, {target_xyz[1]}, {target_xyz[2]}) is unreachable')
                return
            self.get_logger().info('Position reached!')
        
        # Update current position
        self.current_xyz = target_xyz
    
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
