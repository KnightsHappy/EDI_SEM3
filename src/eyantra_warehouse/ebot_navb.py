#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
import math

class eBotNav(Node):
    def __init__(self):
        super().__init__('ebot_nav')

        self._publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscribe = self.create_subscription(Odometry,"/odom",self.odom_callback,10)

        self.waypoints = [[-1.53, -1.95, 1.57 ],[ 0.13, 1.24, 0.00 ],[ 0.38, -3.32, -1.57 ]]
        self.waypoint_count = 0
  
        self.kp_linear = 0.15
        self.kp_angular = 1.0
        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self.is_obstacle_detected = False
        self.turn_once = True 
        
    def odom_callback(self, msg):

        q = msg.pose.pose.orientation
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.current_yaw = yaw 

        self.navigate_to_waypoint()




    def scan_callback(self, msg):
        front_ranges = msg.ranges[345:360] + msg.ranges[0:15]
        self.is_obstacle_detected = min(front_ranges) < 0.5 


    def navigate_to_waypoint(self):

        if self.waypoint_count  >= len(self.waypoints) or self.current_x is None: 
            return
        target_x, target_y, target_yaw = self.waypoints[self.waypoint_count]

        dist_error = math.dist((target_x, target_y), (self.current_x, self.current_y))
        angle_to_target = math.atan2(target_y - self.current_y, target_x - self.current_x)
        yaw_error = angle_to_target - self.current_yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        msg = Twist()
        self.get_logger().info("current yaw"+str(self.current_yaw))
        self.get_logger().info("target _yaw"+ str(target_yaw))

        
        if self.waypoint_count == 1 : #Turning logic for
                    y_intercept = (self.waypoints[self.waypoint_count][0], self.waypoints[self.waypoint_count + 1][1])
                    dist_y_intercept_waypoint = math.dist( (self.waypoints[self.waypoint_count][0],self.waypoints[self.waypoint_count][1]),(y_intercept[0], y_intercept[1]))
                    self.get_logger().info("Dist err:" +str (dist_error))
                    self.get_logger().info("y_intercept " +str (dist_y_intercept_waypoint))
                    if abs(dist_error - 1.5) < 0.1 and self.turn_once == True: 
                        self.get_logger().info("Orientation reached. Moving to next waypoint.")
                        msg.angular.z = self.kp_angular * yaw_error
                        if abs(yaw_error) < math.radians(10):
                            self.get_logger().info("Orientation reached. Moving to next waypoint.")
                            self.turn_once = False
                            
                    else:
                         msg.linear.x = self.kp_linear * dist_error
                         self.get_logger().info("Dist err:" +str (dist_error))

        else: #if not waypoint 1 then move straight until dist error < 0.3 
            
            """if self.is_obstacle_detected: #stopping logic if obstacle detected
                self.get_logger().info("Obstacle detected! Stopping.")
                msg.linear.x = 0.0
                msg.angular.z = 0.0 """
        
        
            if dist_error < 0.3:
                self.get_logger().info("Reached. aligning orientation")
                """msg.linear.x = 0.0
                msg.linear.y = 0.0 
                corrective_yaw_error = target_yaw - self.current_yaw
                corrective_yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
                msg.angular.z = self.kp_angular * corrective_yaw_error"""
            
                self.get_logger().info("currecting ")

                if abs(yaw_error) < math.radians(10):
                    self.get_logger().info("Orientation reached. Moving to next waypoint.")
                    
                    self.waypoint_count += 1
                else:
                    msg.angular.z = self.kp_angular * yaw_error

            else:
                msg.linear.x = self.kp_linear * dist_error
                #msg.angular.z = self.kp_angular * yaw_error

        self._publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = eBotNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()