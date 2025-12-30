"""
Path Executor - Plans and executes smooth paths
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import time
import math
from kinematics import ArmKinematics


class PathExecutorNode(Node):
    def __init__(self):
        super().__init__('path_executor_node')
        
        self.kinematics = ArmKinematics()
        self.xyz_pub = self.create_publisher(Point, 'arm_xyz', 10)
        
        self.get_logger().info('Path Executor ready')
    
    def plan_and_execute(self, start_xyz, end_xyz, max_joint_change=5.0):
        """Plan smooth path and execute it."""
        self.get_logger().info(f'Planning path from {start_xyz} to {end_xyz}')
        
        # Get joint angles for start and end
        start_angles = self.kinematics.inverse_kinematics(*start_xyz)
        end_angles = self.kinematics.inverse_kinematics(*end_xyz)
        
        if not start_angles or not end_angles:
            self.get_logger().error('Position unreachable')
            return False
        
        # Calculate number of waypoints needed
        max_angle_diff = max(abs(end_angles[i] - start_angles[i]) for i in range(6))
        num_steps = max(1, int(math.ceil(max_angle_diff / max_joint_change)))
        
        self.get_logger().info(f'Executing {num_steps + 1} waypoints...')
        
        # Generate and execute waypoints
        for i in range(num_steps + 1):
            t = i / num_steps
            
            # Interpolate joint angles
            interpolated = [start_angles[j] + t * (end_angles[j] - start_angles[j]) 
                          for j in range(6)]
            
            # Calculate XYZ for these angles
            pos = self.kinematics.forward_kinematics(interpolated)
            
            # Publish waypoint
            msg = Point()
            msg.x, msg.y, msg.z = pos['x'], pos['y'], pos['z']
            self.xyz_pub.publish(msg)
            
            time.sleep(0.05)
        
        self.get_logger().info('✓ Path complete')
        return True


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Execute planned path')
    parser.add_argument('--start', nargs=3, type=float, default=[0.0, -31.0, 29.0])
    parser.add_argument('--end', nargs=3, type=float, default=[-31.0, 11.0, 26.0])
    parser.add_argument('--max-change', type=float, default=5.0)
    
    args = parser.parse_args()
    
    rclpy.init()
    node = PathExecutorNode()
    
    time.sleep(1.0)  # Wait for servo_control to be ready
    
    node.plan_and_execute(
        tuple(args.start),
        tuple(args.end),
        args.max_change
    )
    
    time.sleep(1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
