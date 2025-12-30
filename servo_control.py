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
        #1
        # Smooth movement parameters
        self.movement_speed = 2.0  # degrees per step (smaller = smoother)
        self.step_delay = 0.02     # seconds between steps (50Hz update)
        
        # Initialization flag - ignore commands during startup
        self.initialized = False
        
        # Create subscribers BEFORE initialization
        # But callbacks will check self.initialized flag
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'arm_command',
            self.arm_callback,
            10
        )
        
        self.demo_subscription = self.create_subscription(
            String,
            'arm_demo',
            self.demo_callback,
            10
        )
        
        self.xyz_subscription = self.create_subscription(
            Point,
            'arm_xyz',
            self.xyz_callback,
            10
        )
        
        self.obstacle_subscription = self.create_subscription(
            String,
            'arm_obstacles',
            self.obstacle_callback,
            10
        )
        
        # Wait a moment for topics to settle
        self.get_logger().info('Waiting for topics to settle...')
        time.sleep(0.5)
        
        # Initialize all servos to center position SLOWLY
        self.get_logger().info('Initializing robotic arm to center position...')
        self.get_logger().info('Please wait - moving servos slowly to prevent jerky motion')
        
        # Move to center very slowly
        for channel in range(self.NUM_SERVOS):
            self.get_logger().info(f'  Initializing servo {channel} ({self.servo_names[channel]})...')
            # Start from current angle (assume 90) and move slowly
            current = 90.0
            target = 90.0
            
            # Move in 1-degree increments
            steps = 20  # Take 20 steps to reach target
            for step in range(steps + 1):
                angle = current + (target - current) * (step / steps)
                self.set_servo_angle(channel, angle)
                time.sleep(0.05)  # 50ms between steps
            
            self.current_angles[channel] = target
        
        # Now we're ready to accept commands
        self.initialized = True
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Robotic Arm node READY!')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Listening on: /arm_command, /arm_demo, /arm_xyz, /arm_obstacles')
        self.get_logger().info(f'Controlling {self.NUM_SERVOS} servos on channels 0-{self.NUM_SERVOS-1}')
        self.get_logger().info('Servo types: Ch0-1=MG996R, Ch2-5=DS3218')
        self.get_logger().info('Path planning: ENABLED')
        self.get_logger().info(f'Movement speed: {self.movement_speed}°/step, {self.step_delay*1000:.0f}ms delay')
        self.get_logger().info('All servos initialized to 90° (center)')
        self.get_logger().info('Ready to accept commands!')
        self.get_logger().info('=' * 60)
    
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
    
    def smooth_move_to_angles(self, target_angles):
        """
        Smoothly move servos from current angles to target angles.
        
        Args:
            target_angles (list): Target angles for all 6 servos
        """
        # Calculate maximum angle difference
        max_diff = 0
        for i in range(self.NUM_SERVOS):
            diff = abs(target_angles[i] - self.current_angles[i])
            if diff > max_diff:
                max_diff = diff
        
        if max_diff == 0:
            return  # Already at target
        
        # Calculate number of steps needed
        num_steps = int(max_diff / self.movement_speed)
        if num_steps == 0:
            num_steps = 1
        
        # Interpolate and move
        for step in range(num_steps + 1):
            t = step / num_steps  # 0 to 1
            
            for channel in range(self.NUM_SERVOS):
                # Linear interpolation
                angle = self.current_angles[channel] + t * (target_angles[channel] - self.current_angles[channel])
                self.set_servo_angle(channel, angle)
            
            # Small delay between steps for smooth motion
            if step < num_steps:
                time.sleep(self.step_delay)
        
        # Update current angles
        self.current_angles = list(target_angles)
    
    def move_to_waypoint(self, x, y, z):
        """Move to a single XYZ waypoint with smooth motion."""
        # Calculate joint angles
        logical_angles = self.arm_kinematics.inverse_kinematics(x, y, z)
        
        if logical_angles is None:
            return False
        
        # Map to physical channels
        target_angles = [
            logical_angles[5],  # Gripper -> Channel 0
            logical_angles[4],  # Wrist Roll -> Channel 1
            logical_angles[3],  # Wrist Pitch -> Channel 2
            logical_angles[2],  # Elbow -> Channel 3
            logical_angles[1],  # Shoulder -> Channel 4
            logical_angles[0],  # Base -> Channel 5
        ]
        
        # Smoothly move to target angles
        self.smooth_move_to_angles(target_angles)
        
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
        # Ignore commands during initialization
        if not self.initialized:
            self.get_logger().warn('Ignoring demo command - node still initializing')
            return
        
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
        # Ignore commands during initialization
        if not self.initialized:
            self.get_logger().warn('Ignoring command - node still initializing')
            return
        
        self.get_logger().info('=== ARM CALLBACK TRIGGERED ===')
        
        if len(msg.data) != self.NUM_SERVOS:
            self.get_logger().error(f'Expected {self.NUM_SERVOS} angles, got {len(msg.data)}')
            return
        
        self.get_logger().info(f'Received arm command: {[f"{a:.1f}°" for a in msg.data]}')
        
        # Smoothly move to commanded angles
        self.smooth_move_to_angles(list(msg.data))
        
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
        # Ignore commands during initialization
        if not self.initialized:
            self.get_logger().warn('Ignoring obstacle command - node still initializing')
            return
        
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
        # Ignore commands during initialization
        if not self.initialized:
            self.get_logger().warn('Ignoring XYZ command - node still initializing')
            return
        
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
            
            # Execute path with smooth motion
            self.get_logger().info(f'Executing path with {len(path)} waypoints...')
            for i, waypoint in enumerate(path):
                success = self.move_to_waypoint(*waypoint)
                if not success:
                    self.get_logger().error(f'Failed at waypoint {i}')
                    return
                # No additional delay needed - smooth_move_to_angles handles it
            
            self.get_logger().info('Path execution complete!')
        
        else:
            # Direct movement (no path planning) with smooth motion
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
