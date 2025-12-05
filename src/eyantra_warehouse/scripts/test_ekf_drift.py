#!/usr/bin/env python3
"""
Automated EKF Drift Testing Script
Runs diagnostic checks and navigation test automatically
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import time
import math


class EKFDriftTester(Node):
    def __init__(self):
        super().__init__('ekf_drift_tester')
        
        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Subscribers
        self.raw_odom_sub = self.create_subscription(
            Odometry, '/odom', self.raw_odom_callback, 10)
        self.filtered_odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.filtered_odom_callback, 10)
        
        # Data storage
        self.raw_odom_data = []
        self.filtered_odom_data = []
        self.test_started = False
        self.goal_sent = False
        
        # Test parameters
        self.goal_x = -0.5
        self.goal_y = -6.6
        
        self.get_logger().info('=== EKF Drift Tester Started ===')
        self.get_logger().info('Waiting 15 seconds for system initialization...')
        
        # Start test after delay
        self.timer = self.create_timer(15.0, self.start_test)
    
    def raw_odom_callback(self, msg):
        if self.test_started:
            pos = msg.pose.pose.position
            self.raw_odom_data.append((pos.x, pos.y))
    
    def filtered_odom_callback(self, msg):
        if self.test_started:
            pos = msg.pose.pose.position
            self.filtered_odom_data.append((pos.x, pos.y))
    
    def start_test(self):
        self.timer.cancel()
        self.get_logger().info('\n=== Starting Drift Test ===')
        
        # Record initial positions
        time.sleep(1.0)
        self.test_started = True
        time.sleep(0.5)
        
        if self.raw_odom_data and self.filtered_odom_data:
            raw_start = self.raw_odom_data[0]
            filt_start = self.filtered_odom_data[0]
            
            self.get_logger().info(f'\nInitial Positions:')
            self.get_logger().info(f'  Raw odom:      x={raw_start[0]:.3f}, y={raw_start[1]:.3f}')
            self.get_logger().info(f'  Filtered odom: x={filt_start[0]:.3f}, y={filt_start[1]:.3f}')
            self.get_logger().info(f'  Difference:    x={abs(raw_start[0]-filt_start[0]):.3f}, y={abs(raw_start[1]-filt_start[1]):.3f}')
        
        # Send navigation goal
        self.get_logger().info(f'\nSending navigation goal to ({self.goal_x}, {self.goal_y})...')
        self.send_goal()
        
        # Monitor for 30 seconds
        self.monitor_timer = self.create_timer(5.0, self.print_status)
        self.end_timer = self.create_timer(30.0, self.end_test)
    
    def send_goal(self):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = self.goal_x
        goal.pose.position.y = self.goal_y
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal)
        self.goal_sent = True
        self.get_logger().info('Goal published!')
    
    def print_status(self):
        if len(self.raw_odom_data) > 1 and len(self.filtered_odom_data) > 1:
            raw_curr = self.raw_odom_data[-1]
            filt_curr = self.filtered_odom_data[-1]
            
            # Calculate errors from goal
            raw_error = math.sqrt((raw_curr[0] - self.goal_x)**2 + 
                                 (raw_curr[1] - self.goal_y)**2)
            filt_error = math.sqrt((filt_curr[0] - self.goal_x)**2 + 
                                  (filt_curr[1] - self.goal_y)**2)
            
            self.get_logger().info(f'\nCurrent Status:')
            self.get_logger().info(f'  Raw odom:      x={raw_curr[0]:.3f}, y={raw_curr[1]:.3f} | Error: {raw_error:.3f}m')
            self.get_logger().info(f'  Filtered odom: x={filt_curr[0]:.3f}, y={filt_curr[1]:.3f} | Error: {filt_error:.3f}m')
            self.get_logger().info(f'  Drift diff:    {abs(raw_error - filt_error):.3f}m')
    
    def end_test(self):
        self.monitor_timer.cancel()
        self.end_timer.cancel()
        
        self.get_logger().info('\n=== Test Complete ===')
        
        if len(self.raw_odom_data) > 1 and len(self.filtered_odom_data) > 1:
            raw_final = self.raw_odom_data[-1]
            filt_final = self.filtered_odom_data[-1]
            raw_start = self.raw_odom_data[0]
            
            # Calculate final errors
            raw_error = math.sqrt((raw_final[0] - self.goal_x)**2 + 
                                 (raw_final[1] - self.goal_y)**2)
            filt_error = math.sqrt((filt_final[0] - self.goal_x)**2 + 
                                  (filt_final[1] - self.goal_y)**2)
            
            # Calculate distance traveled
            dist_traveled = math.sqrt((raw_final[0] - raw_start[0])**2 + 
                                     (raw_final[1] - raw_start[1])**2)
            
            # Calculate drift percentages
            raw_drift_pct = (raw_error / dist_traveled * 100) if dist_traveled > 0 else 0
            filt_drift_pct = (filt_error / dist_traveled * 100) if dist_traveled > 0 else 0
            
            self.get_logger().info(f'\nFinal Results:')
            self.get_logger().info(f'  Distance traveled: {dist_traveled:.3f}m')
            self.get_logger().info(f'  Goal: ({self.goal_x}, {self.goal_y})')
            self.get_logger().info(f'\n  Raw Odometry:')
            self.get_logger().info(f'    Final position: ({raw_final[0]:.3f}, {raw_final[1]:.3f})')
            self.get_logger().info(f'    Error from goal: {raw_error:.3f}m ({raw_drift_pct:.1f}%)')
            self.get_logger().info(f'\n  Filtered Odometry (EKF):')
            self.get_logger().info(f'    Final position: ({filt_final[0]:.3f}, {filt_final[1]:.3f})')
            self.get_logger().info(f'    Error from goal: {filt_error:.3f}m ({filt_drift_pct:.1f}%)')
            
            improvement = raw_error - filt_error
            improvement_pct = (improvement / raw_error * 100) if raw_error > 0 else 0
            
            self.get_logger().info(f'\n  EKF Improvement:')
            if improvement > 0:
                self.get_logger().info(f'    Drift reduced by: {improvement:.3f}m ({improvement_pct:.1f}%)')
                self.get_logger().info(f'    ✅ EKF is working!')
            else:
                self.get_logger().info(f'    ⚠️  EKF not improving drift (worse by {abs(improvement):.3f}m)')
        
        self.get_logger().info('\n=== Test Ended ===')
        self.get_logger().info('You can now manually test navigation or restart the test.')


def main(args=None):
    rclpy.init(args=args)
    tester = EKFDriftTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
