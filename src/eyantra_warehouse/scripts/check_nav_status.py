#!/usr/bin/env python3
"""
Navigation and Localization Diagnostic Script
Checks the status of Nav2 stack, transforms, and sensor data
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import sys
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class NavDiagnostic(Node):
    def __init__(self):
        super().__init__('nav_diagnostic')
        
        # Status flags
        self.checks = {
            'amcl_pose': False,
            'odom': False,
            'scan': False,
            'map': False,
            'tf_map_odom': False,
            'tf_odom_base': False,
            'tf_map_base': False
        }
        
        # QoS profile for best effort
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Timer to check TF
        self.timer = self.create_timer(1.0, self.check_transforms)
        
        # Timer to print status
        self.status_timer = self.create_timer(3.0, self.print_status)
        
        self.get_logger().info('Navigation Diagnostic Node Started')
        self.get_logger().info('Checking navigation stack status...')
    
    def amcl_callback(self, msg):
        if not self.checks['amcl_pose']:
            self.get_logger().info('✓ AMCL pose is being published')
            self.checks['amcl_pose'] = True
    
    def odom_callback(self, msg):
        if not self.checks['odom']:
            self.get_logger().info('✓ Odometry is being published')
            self.checks['odom'] = True
    
    def scan_callback(self, msg):
        if not self.checks['scan']:
            self.get_logger().info(f'✓ Laser scan is being published ({len(msg.ranges)} points)')
            self.checks['scan'] = True
    
    def map_callback(self, msg):
        if not self.checks['map']:
            self.get_logger().info(f'✓ Map is being published ({msg.info.width}x{msg.info.height})')
            self.checks['map'] = True
    
    def check_transforms(self):
        try:
            # Check map -> odom
            if not self.checks['tf_map_odom']:
                trans = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
                self.get_logger().info('✓ Transform map->odom is available')
                self.checks['tf_map_odom'] = True
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass
        
        try:
            # Check odom -> base_link
            if not self.checks['tf_odom_base']:
                trans = self.tf_buffer.lookup_transform('odom', 'ebot_base_link', rclpy.time.Time())
                self.get_logger().info('✓ Transform odom->ebot_base_link is available')
                self.checks['tf_odom_base'] = True
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass
        
        try:
            # Check map -> base_link (full chain)
            if not self.checks['tf_map_base']:
                trans = self.tf_buffer.lookup_transform('map', 'ebot_base_link', rclpy.time.Time())
                self.get_logger().info('✓ Transform map->ebot_base_link is available (full chain)')
                self.checks['tf_map_base'] = True
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass
    
    def print_status(self):
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('NAVIGATION STACK STATUS:')
        self.get_logger().info('='*60)
        
        # Check which components are working
        working = sum(self.checks.values())
        total = len(self.checks)
        
        for check, status in self.checks.items():
            symbol = '✓' if status else '✗'
            self.get_logger().info(f'{symbol} {check}: {"OK" if status else "NOT DETECTED"}')
        
        self.get_logger().info('='*60)
        self.get_logger().info(f'Status: {working}/{total} components detected')
        
        # Provide recommendations
        if not self.checks['amcl_pose']:
            self.get_logger().warn('⚠ AMCL not running - localization may not be working')
            self.get_logger().warn('  → Launch Nav2 stack with: ros2 launch eyantra_warehouse nav2.launch.py')
        
        if not self.checks['map']:
            self.get_logger().warn('⚠ Map not published - navigation requires a map')
            self.get_logger().warn('  → Check if map_server is running')
        
        if not self.checks['scan']:
            self.get_logger().warn('⚠ Laser scan not available - check sensor bridge')
        
        if not self.checks['odom']:
            self.get_logger().warn('⚠ Odometry not available - check robot bridge')
        
        if self.checks['tf_map_odom'] and not self.checks['amcl_pose']:
            self.get_logger().warn('⚠ map->odom transform exists but AMCL not running')
            self.get_logger().warn('  → This might be a static transform, not proper localization')
        
        self.get_logger().info('='*60 + '\n')

def main(args=None):
    rclpy.init(args=args)
    node = NavDiagnostic()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
