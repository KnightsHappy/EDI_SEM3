#!/usr/bin/env python3
"""
Valid Navigation Goals Helper
Shows valid coordinates within the map boundaries
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math


class ValidGoalsHelper(Node):
    def __init__(self):
        super().__init__('valid_goals_helper')
        
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Map info from campus_map.yaml
        # origin: [-4.39, -7.35, 0]
        # resolution: 0.05
        # size: 119x260 cells (from error message)
        
        map_origin_x = -4.39
        map_origin_y = -7.35
        resolution = 0.05
        width_cells = 119
        height_cells = 260
        
        # Calculate map boundaries
        max_x = map_origin_x + (width_cells * resolution)
        max_y = map_origin_y + (height_cells * resolution)
        
        self.get_logger().info('=== Valid Navigation Area ===')
        self.get_logger().info(f'Map boundaries:')
        self.get_logger().info(f'  X range: {map_origin_x:.2f} to {max_x:.2f}')
        self.get_logger().info(f'  Y range: {map_origin_y:.2f} to {max_y:.2f}')
        self.get_logger().info(f'\nRobot starts at: (-1.53, -6.62)')
        self.get_logger().info(f'\nSuggested test goals (within map):')
        self.get_logger().info(f'  1. Forward:     x=-1.5, y=-5.0')
        self.get_logger().info(f'  2. Right:       x=-2.5, y=-6.6')
        self.get_logger().info(f'  3. Left:        x=-0.5, y=-6.6')
        self.get_logger().info(f'  4. Far forward: x=-1.5, y=-3.0')
        self.get_logger().info(f'\nSending goal #3 (left) in 3 seconds...')
        
        # Send a valid goal after delay
        self.timer = self.create_timer(3.0, self.send_valid_goal)
    
    def send_valid_goal(self):
        self.timer.cancel()
        
        # Send a goal that's definitely within the map
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        
        goal.pose.position.x = -0.5  # Within map bounds
        goal.pose.position.y = -6.6  # Within map bounds
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal)
        self.get_logger().info(f'\n✅ Goal sent: x=-0.5, y=-6.6')
        self.get_logger().info('Watch RViz for purple path and robot movement!')


def main(args=None):
    rclpy.init(args=args)
    helper = ValidGoalsHelper()
    
    try:
        rclpy.spin(helper)
    except KeyboardInterrupt:
        pass
    finally:
        helper.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
