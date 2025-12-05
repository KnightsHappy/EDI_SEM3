#!/usr/bin/env python3
"""
Test script to send a navigation goal to Nav2.
This helps diagnose if the navigation stack is working.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math


class NavGoalTester(Node):
    def __init__(self):
        super().__init__('nav_goal_tester')
        
        # Publisher for navigation goals
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )
        
        # Wait a bit for connections
        self.timer = self.create_timer(2.0, self.send_test_goal)
        self.goal_sent = False
        
        self.get_logger().info('Nav goal tester started. Will send test goal in 2 seconds...')
    
    def send_test_goal(self):
        if self.goal_sent:
            return
            
        # Create a test goal - adjust coordinates as needed
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        
        # Set a goal position (1 meter forward from spawn)
        goal.pose.position.x = -0.5  # Adjust based on your map
        goal.pose.position.y = -6.6
        goal.pose.position.z = 0.0
        
        # Orientation (facing forward)
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        
        # Publish the goal
        self.goal_pub.publish(goal)
        self.get_logger().info(f'Published test goal to ({goal.pose.position.x}, {goal.pose.position.y})')
        self.get_logger().info('Check if robot starts moving and if /plan topic shows a path')
        
        self.goal_sent = True
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = NavGoalTester()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
