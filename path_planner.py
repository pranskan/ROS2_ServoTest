"""
Path Planning for Robotic Arm
------------------------------
Plans smooth paths between two XYZ positions.
"""
import math
import argparse
from kinematics import ArmKinematics


class PathPlanner:
    """
    Simple path planner that creates smooth waypoint-based movements.
    """
    
    def __init__(self, arm_kinematics):
        """
        Initialize path planner.
        
        Args:
            arm_kinematics: ArmKinematics instance
        """
        self.kinematics = arm_kinematics
        
        # Define obstacle zones (optional, for future use)
        self.obstacles = {}
    
    def plan_smooth_path(self, start_xyz, end_xyz, max_joint_change=5.0):
        """
        Plan a smooth path with small joint angle changes.
        
        Args:
            start_xyz (tuple): Starting (x, y, z) in cm
            end_xyz (tuple): Ending (x, y, z) in cm
            max_joint_change (float): Max degrees change per waypoint (default 5°)
        
        Returns:
            list: List of waypoints [(x, y, z), ...] or None if no path
        """
        x0, y0, z0 = start_xyz
        x1, y1, z1 = end_xyz
        
        print(f"\n🔍 Planning smooth path...")
        print(f"   From: ({x0:.2f}, {y0:.2f}, {z0:.2f}) cm")
        print(f"   To:   ({x1:.2f}, {y1:.2f}, {z1:.2f}) cm")
        print(f"   Max joint change: {max_joint_change}° per step")
        
        # Calculate joint angles for start and end positions
        start_angles = self.kinematics.inverse_kinematics(x0, y0, z0)
        end_angles = self.kinematics.inverse_kinematics(x1, y1, z1)
        
        if start_angles is None:
            print(f"❌ Start position unreachable")
            return None
        
        if end_angles is None:
            print(f"❌ End position unreachable")
            return None
        
        # Calculate how many steps we need based on max joint change
        max_angle_diff = 0
        for i in range(6):
            angle_diff = abs(end_angles[i] - start_angles[i])
            if angle_diff > max_angle_diff:
                max_angle_diff = angle_diff
        
        # Number of steps needed (round up)
        num_steps = int(math.ceil(max_angle_diff / max_joint_change))
        if num_steps == 0:
            num_steps = 1
        
        print(f"   Max angle change needed: {max_angle_diff:.2f}°")
        print(f"   Generating {num_steps + 1} waypoints...")
        
        waypoints = []
        
        # Generate waypoints by interpolating joint angles
        for i in range(num_steps + 1):
            t = i / num_steps  # Parameter from 0 to 1
            
            # Interpolate each joint angle
            interpolated_angles = []
            for j in range(6):
                angle = start_angles[j] + t * (end_angles[j] - start_angles[j])
                interpolated_angles.append(angle)
            
            # Calculate XYZ position for these joint angles (forward kinematics)
            position = self.kinematics.forward_kinematics(interpolated_angles)
            waypoint = (position['x'], position['y'], position['z'])
            waypoints.append(waypoint)
            
            # Show progress
            if i % max(1, num_steps // 5) == 0 or i == num_steps:
                print(f"   Waypoint {i}/{num_steps}: ({waypoint[0]:.2f}, {waypoint[1]:.2f}, {waypoint[2]:.2f}) cm")
        
        print(f"✓ Path created with {len(waypoints)} waypoints")
        return waypoints
    
    def plan_linear_path(self, start_xyz, end_xyz, num_waypoints=20):
        """
        Plan a straight-line path in Cartesian space.
        
        Args:
            start_xyz (tuple): Starting (x, y, z) in cm
            end_xyz (tuple): Ending (x, y, z) in cm
            num_waypoints (int): Number of waypoints
        
        Returns:
            list: List of waypoints or None if unreachable
        """
        x0, y0, z0 = start_xyz
        x1, y1, z1 = end_xyz
        
        print(f"\n🔍 Planning linear path...")
        print(f"   From: ({x0:.2f}, {y0:.2f}, {z0:.2f}) cm")
        print(f"   To:   ({x1:.2f}, {y1:.2f}, {z1:.2f}) cm")
        print(f"   Creating {num_waypoints} waypoints...")
        
        waypoints = []
        
        for i in range(num_waypoints + 1):
            t = i / num_waypoints
            
            # Linear interpolation in Cartesian space
            x = x0 + t * (x1 - x0)
            y = y0 + t * (y1 - y0)
            z = z0 + t * (z1 - z0)
            
            # Check if this position is reachable
            reachable, reason = self.kinematics.is_position_reachable(x, y, z)
            if not reachable:
                print(f"❌ Waypoint {i}/{num_waypoints} unreachable: {reason}")
                return None
            
            # Check if we can solve IK for this position
            angles = self.kinematics.inverse_kinematics(x, y, z)
            if angles is None:
                print(f"❌ No IK solution for waypoint {i}/{num_waypoints}")
                return None
            
            waypoints.append((x, y, z))
            
            # Show progress
            if i % max(1, num_waypoints // 5) == 0 or i == num_waypoints:
                print(f"   Waypoint {i}/{num_waypoints}: ({x:.2f}, {y:.2f}, {z:.2f}) cm")
        
        print(f"✓ Linear path created with {len(waypoints)} waypoints")
        return waypoints
    
    def plan_best_path(self, start_xyz, end_xyz, max_joint_change=5.0):
        """
        Plan the best smooth path.
        
        Args:
            start_xyz (tuple): Starting position
            end_xyz (tuple): Ending position
            max_joint_change (float): Max degrees per waypoint
        
        Returns:
            list: Best path waypoints or None
        """
        # Use smooth path planning (joint space interpolation)
        return self.plan_smooth_path(start_xyz, end_xyz, max_joint_change)


def test_path_planner():
    """Test the path planner with command-line arguments."""
    parser = argparse.ArgumentParser(
        description='Test path planner with custom start and end positions',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Default test case
  python3 path_planner.py
  
  # Custom positions
  python3 path_planner.py --start 0 -31.07 29.15 --end -31.76 11.56 26.34
  
  # Different max joint change
  python3 path_planner.py --start 10 20 30 --end -10 -20 25 --max-change 10
  
  # More waypoints (smaller steps)
  python3 path_planner.py --start 0 0 30 --end 20 20 30 --max-change 3
        """
    )
    
    parser.add_argument(
        '--start', 
        nargs=3, 
        type=float,
        metavar=('X', 'Y', 'Z'),
        default=[-0.00, -31.07, 29.15],
        help='Start position (x y z) in cm (default: -0.00 -31.07 29.15)'
    )
    
    parser.add_argument(
        '--end', 
        nargs=3, 
        type=float,
        metavar=('X', 'Y', 'Z'),
        default=[-31.76, 11.56, 26.34],
        help='End position (x y z) in cm (default: -31.76 11.56 26.34)'
    )
    
    parser.add_argument(
        '--max-change',
        type=float,
        default=5.0,
        metavar='DEGREES',
        help='Maximum joint angle change per waypoint in degrees (default: 5.0)'
    )
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("PATH PLANNER TEST")
    print("=" * 60)
    
    # Create kinematics and planner
    kinematics = ArmKinematics()
    planner = PathPlanner(kinematics)
    
    # Use command-line arguments
    start_pos = tuple(args.start)
    end_pos = tuple(args.end)
    max_change = args.max_change
    
    print("\n" + "=" * 60)
    print(f"Test: Move with {max_change}° max joint changes")
    
    path = planner.plan_best_path(start_pos, end_pos, max_joint_change=max_change)
    
    if path:
        print(f"\n✓ Path with {len(path)} waypoints:")
        print(f"\n   Waypoint breakdown:")
        for i, waypoint in enumerate(path):
            print(f"     {i:2d}: ({waypoint[0]:7.2f}, {waypoint[1]:7.2f}, {waypoint[2]:7.2f}) cm")
        
        # Calculate total distance
        total_dist = 0
        for i in range(len(path) - 1):
            dx = path[i+1][0] - path[i][0]
            dy = path[i+1][1] - path[i][1]
            dz = path[i+1][2] - path[i][2]
            total_dist += math.sqrt(dx**2 + dy**2 + dz**2)
        
        print(f"\n   Total path length: {total_dist:.2f} cm")
        print(f"   Average step size: {total_dist / (len(path) - 1):.2f} cm")
    else:
        print("\n✗ Failed to find path")
    
    print("\n" + "=" * 60)


if __name__ == "__main__":
    test_path_planner()
