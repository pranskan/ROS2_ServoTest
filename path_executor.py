"""
ROS2 Path Executor Node
-----------------------
Executes planned paths by publishing waypoints to servo_control node.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import time
from kinematics import ArmKinematics
from path_planner import PathPlanner


class PathExecutorNode(Node):
    def __init__(self):
        super().__init__('path_executor_node')
        
        # Initialize kinematics and planner
        self.kinematics = ArmKinematics()
        self.planner = PathPlanner(self.kinematics)
        
        # Publisher for XYZ commands
        self.xyz_pub = self.create_publisher(Point, 'arm_xyz', 10)
        
        # Track current position
        self.current_xyz = (0.0, -31.07, 29.15)  # Default start position
        
        self.get_logger().info('Path Executor Node started')
        self.get_logger().info('Publishing to: /arm_xyz')
        
    def execute_path(self, start_xyz, end_xyz, max_joint_change=5.0, delay=0.05):
        """
        Plan and execute a path.
        
        Args:
            start_xyz (tuple): Starting position (x, y, z)
            end_xyz (tuple): Target position (x, y, z)
            max_joint_change (float): Max degrees per waypoint
            delay (float): Delay between waypoints in seconds (default: 0.05)
        """
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Planning path from {start_xyz} to {end_xyz}')
        
        # Plan the path
        waypoints = self.planner.plan_best_path(start_xyz, end_xyz, max_joint_change)
        
        if waypoints is None:
            self.get_logger().error('Failed to plan path!')
            return False
        
        self.get_logger().info(f'Path created with {len(waypoints)} waypoints')
        self.get_logger().info('Executing path...')
        
        # Execute each waypoint
        for i, (x, y, z) in enumerate(waypoints):
            msg = Point()
            msg.x = x
            msg.y = y
            msg.z = z
            
            self.get_logger().info(f'  Waypoint {i+1}/{len(waypoints)}: ({x:.2f}, {y:.2f}, {z:.2f})')
            self.xyz_pub.publish(msg)
            
            # Reduced delay since servo_control now handles smooth motion
            time.sleep(delay)
        
        self.get_logger().info('✓ Path execution complete!')
        self.get_logger().info('=' * 60)
        
        # Update current position
        self.current_xyz = end_xyz
        return True


def main():
    import argparse
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Execute a planned path')
    parser.add_argument('--start', nargs=3, type=float, 
                       default=[0.0, -31.07, 29.15],
                       help='Start position (x y z)')
    parser.add_argument('--end', nargs=3, type=float,
                       default=[-31.76, 11.56, 26.34],
                       help='End position (x y z)')
    parser.add_argument('--max-change', type=float, default=5.0,
                       help='Max joint angle change per waypoint (degrees)')
    parser.add_argument('--delay', type=float, default=0.05,
                       help='Delay between waypoints (seconds, default: 0.05)')
    
    args = parser.parse_args()
    
    rclpy.init()
    node = PathExecutorNode()
    
    # Give servo_control time to start
    time.sleep(1.0)
    
    # Execute the path
    start_pos = tuple(args.start)
    end_pos = tuple(args.end)
    
    node.execute_path(start_pos, end_pos, 
                     max_joint_change=args.max_change,
                     delay=args.delay)
    
    # Keep node alive briefly
    time.sleep(2.0)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
