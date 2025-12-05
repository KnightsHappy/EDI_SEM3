#!/usr/bin/env python3
"""
Simple GUI to send navigation goals to Nav2.
Use this as an alternative to the RViz Nav2 Goal tool.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tkinter as tk
from tkinter import ttk
import math


class NavGoalGUI(Node):
    def __init__(self, root):
        super().__init__('nav_goal_gui')
        self.root = root
        self.root.title("Nav2 Goal Sender")
        
        # Publisher for navigation goals
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Create GUI
        self.create_widgets()
        
        self.get_logger().info('Nav Goal GUI started')
    
    def create_widgets(self):
        # Instructions
        ttk.Label(self.root, text="Send Navigation Goals to Nav2", 
                 font=('Arial', 14, 'bold')).grid(row=0, column=0, columnspan=2, pady=10)
        
        # X coordinate
        ttk.Label(self.root, text="X Position:").grid(row=1, column=0, sticky='e', padx=5, pady=5)
        self.x_entry = ttk.Entry(self.root, width=15)
        self.x_entry.insert(0, "-0.5")
        self.x_entry.grid(row=1, column=1, padx=5, pady=5)
        
        # Y coordinate
        ttk.Label(self.root, text="Y Position:").grid(row=2, column=0, sticky='e', padx=5, pady=5)
        self.y_entry = ttk.Entry(self.root, width=15)
        self.y_entry.insert(0, "-6.6")
        self.y_entry.grid(row=2, column=1, padx=5, pady=5)
        
        # Yaw angle
        ttk.Label(self.root, text="Yaw (degrees):").grid(row=3, column=0, sticky='e', padx=5, pady=5)
        self.yaw_entry = ttk.Entry(self.root, width=15)
        self.yaw_entry.insert(0, "0")
        self.yaw_entry.grid(row=3, column=1, padx=5, pady=5)
        
        # Send button
        self.send_btn = ttk.Button(self.root, text="Send Goal", command=self.send_goal)
        self.send_btn.grid(row=4, column=0, columnspan=2, pady=15)
        
        # Status label
        self.status_label = ttk.Label(self.root, text="Ready", foreground="blue")
        self.status_label.grid(row=5, column=0, columnspan=2, pady=5)
        
        # Preset buttons
        ttk.Label(self.root, text="Quick Goals:", font=('Arial', 10, 'bold')).grid(
            row=6, column=0, columnspan=2, pady=(15, 5))
        
        preset_frame = ttk.Frame(self.root)
        preset_frame.grid(row=7, column=0, columnspan=2, pady=5)
        
        ttk.Button(preset_frame, text="Forward 1m", 
                  command=lambda: self.set_preset(-0.5, -6.6, 0)).pack(side='left', padx=2)
        ttk.Button(preset_frame, text="Forward 2m", 
                  command=lambda: self.set_preset(0.5, -6.6, 0)).pack(side='left', padx=2)
        ttk.Button(preset_frame, text="Turn Right", 
                  command=lambda: self.set_preset(-1.5, -6.6, -90)).pack(side='left', padx=2)
    
    def set_preset(self, x, y, yaw):
        self.x_entry.delete(0, tk.END)
        self.x_entry.insert(0, str(x))
        self.y_entry.delete(0, tk.END)
        self.y_entry.insert(0, str(y))
        self.yaw_entry.delete(0, tk.END)
        self.yaw_entry.insert(0, str(yaw))
    
    def send_goal(self):
        try:
            x = float(self.x_entry.get())
            y = float(self.y_entry.get())
            yaw_deg = float(self.yaw_entry.get())
            yaw_rad = math.radians(yaw_deg)
            
            # Create goal message
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = 'map'
            
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = 0.0
            
            # Convert yaw to quaternion
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = math.sin(yaw_rad / 2.0)
            goal.pose.orientation.w = math.cos(yaw_rad / 2.0)
            
            # Publish goal
            self.goal_pub.publish(goal)
            
            self.status_label.config(text=f"Goal sent: ({x:.2f}, {y:.2f}, {yaw_deg}°)", 
                                    foreground="green")
            self.get_logger().info(f'Sent goal: x={x}, y={y}, yaw={yaw_deg}°')
            
        except ValueError as e:
            self.status_label.config(text="Error: Invalid number format", foreground="red")
            self.get_logger().error(f'Invalid input: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    root = tk.Tk()
    node = NavGoalGUI(root)
    
    def spin_once():
        rclpy.spin_once(node, timeout_sec=0.1)
        root.after(100, spin_once)
    
    root.after(100, spin_once)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
