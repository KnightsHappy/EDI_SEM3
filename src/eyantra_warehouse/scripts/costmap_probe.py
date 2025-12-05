#!/usr/bin/env python3
"""
Costmap Probe Script
Checks the cost value in the global costmap at specific coordinates
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import math

class CostmapProbe(Node):
    def __init__(self):
        super().__init__('costmap_probe')
        
        from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            qos)
        
        self.costmap = None
        self.get_logger().info('Waiting for global costmap...')
        
        # Points to check
        self.points_to_check = [
            (-1.50, -6.60), # Failing Goal (Spawn point)
            (0.5, 2.2),     # User reported fail
            (0.5, 4.0),     # User reported fail
            (-0.96, -4.88)  # Last known robot position
        ]

    def costmap_callback(self, msg):
        self.costmap = msg
        self.get_logger().info(f'Received costmap: {msg.info.width}x{msg.info.height} @ {msg.info.resolution}m/px')
        self.check_points()
        # We only need one check
        rclpy.shutdown()

    def check_points(self):
        if not self.costmap:
            return

        info = self.costmap.info
        data = self.costmap.data
        
        self.get_logger().info('\n=== Costmap Probe Results ===')
        self.get_logger().info(f'Map Origin: ({info.origin.position.x:.2f}, {info.origin.position.y:.2f})')
        
        for x, y in self.points_to_check:
            # Convert world to grid
            mx = int((x - info.origin.position.x) / info.resolution)
            my = int((y - info.origin.position.y) / info.resolution)
            
            if 0 <= mx < info.width and 0 <= my < info.height:
                index = my * info.width + mx
                cost = data[index]
                
                status = "UNKNOWN"
                if cost == 0: status = "FREE"
                elif cost == 100: status = "LETHAL OBSTACLE"
                elif cost == -1: status = "UNKNOWN SPACE"
                elif cost > 0: status = f"INFLATED ({cost})"
                
                self.get_logger().info(f'Point ({x:.2f}, {y:.2f}) -> Grid ({mx}, {my}) -> Cost: {cost} [{status}]')
            else:
                self.get_logger().info(f'Point ({x:.2f}, {y:.2f}) -> OUT OF BOUNDS')

def main(args=None):
    rclpy.init(args=args)
    probe = CostmapProbe()
    try:
        rclpy.spin(probe)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()
