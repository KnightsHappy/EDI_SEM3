#!/usr/bin/env python3
"""
Navigation Monitor Script
Captures logs and status to debug navigation failures in real-time
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rcl_interfaces.msg import Log
from action_msgs.msg import GoalStatusArray

class NavigationMonitor(Node):
    def __init__(self):
        super().__init__('navigation_monitor')
        
        # Subscribe to key topics
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(Path, '/plan', self.plan_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Log, '/rosout', self.log_callback, 100)
        
        self.get_logger().info('=== Navigation Monitor Started ===')
        self.get_logger().info('Waiting for goal...')
        
        self.goal_received = False
        self.plan_received = False

    def goal_callback(self, msg):
        self.goal_received = True
        self.plan_received = False
        self.get_logger().info(f'\n[GOAL RECEIVED] Target: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

    def plan_callback(self, msg):
        self.plan_received = True
        self.get_logger().info(f'[PLAN SUCCESS] Path generated with {len(msg.poses)} poses')

    def cmd_vel_callback(self, msg):
        # Only log if we recently got a goal/plan to avoid spam
        if self.goal_received and (abs(msg.linear.x) > 0 or abs(msg.angular.z) > 0):
            # self.get_logger().info(f'[MOVING] v={msg.linear.x:.2f}, w={msg.angular.z:.2f}')
            pass

    def log_callback(self, msg):
        # Filter for relevant nodes and severity (WARN=30, ERROR=40)
        if msg.level >= 30 and ('planner' in msg.name or 'bt_navigator' in msg.name or 'costmap' in msg.name):
            level_str = 'WARN' if msg.level == 30 else 'ERROR'
            self.get_logger().info(f'[{msg.name}] [{level_str}] {msg.msg}')

def main(args=None):
    rclpy.init(args=args)
    monitor = NavigationMonitor()
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
