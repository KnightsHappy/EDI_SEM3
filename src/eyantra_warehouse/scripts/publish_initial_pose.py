#!/usr/bin/env python3
"""
Publish initial pose to AMCL to match robot spawn position.
This allows AMCL to localize immediately without manual pose estimate.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        
        # Publisher for initial pose
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        # Wait a bit for AMCL to be ready, then publish
        self.timer = self.create_timer(2.0, self.publish_initial_pose)
        self.published = False
        
        self.get_logger().info('Initial pose publisher node started')
    
    def publish_initial_pose(self):
        if self.published:
            return
            
        # Create pose message matching spawn position
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        # Position matches spawn coordinates
        pose_msg.pose.pose.position.x = -1.5339
        pose_msg.pose.pose.position.y = -6.6156
        pose_msg.pose.pose.position.z = 0.0
        
        # Orientation: yaw = 1.57 rad (90 degrees)
        yaw = 1.57
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # Covariance matrix (6x6 = 36 elements)
        # Small values indicate high confidence in initial pose
        pose_msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,  # x variance
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,  # y variance
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # z variance (not used in 2D)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # roll variance (not used in 2D)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # pitch variance (not used in 2D)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.07   # yaw variance
        ]
        
        # Publish the pose
        self.pose_pub.publish(pose_msg)
        self.get_logger().info(f'Published initial pose: x={pose_msg.pose.pose.position.x}, '
                              f'y={pose_msg.pose.pose.position.y}, yaw={yaw}')
        
        self.published = True
        self.timer.cancel()  # Stop the timer after publishing once


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
