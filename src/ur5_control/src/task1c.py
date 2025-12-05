#!/usr/bin/env python3
"""
******************************************************************************************
*
* [TASK 1C] - Arm Manipulation with Servoing (Fixed Version + P2 Static TF + Orientation Fix)
* Author: Nihit Shevade
*
* Description:
* This version fixes the "stuck" issue at major waypoints and the 180° orientation flip at P2_modified.
* The arm moves: Start → P1 → P3 → Start → P2_modified,
* using a stable PD-based Cartesian servoing controller.
* A static TF frame is broadcasted at P2_modified for RViz visualization.
*
******************************************************************************************
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformException, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp


class ArmWaypointController(Node):
    def __init__(self):
        super().__init__('arm_waypoint_controller')

        # --- Parameters ---
        self.TARGET_FRAME = 'tool0'
        self.REFERENCE_FRAME = 'base_link'
        self.CONTROL_RATE = 100.0
        self.NUM_INTER_STEPS = 20
        self.dt = 1.0 / self.CONTROL_RATE

        # --- Major Waypoints ---
        self.p1 = {'pos': np.array([-0.214, -0.532, 0.557]),
                   'quat': np.array([0.707, 0.028, 0.034, 0.707])}

        self.p3 = {'pos': np.array([-0.806, 0.010, 0.182]),
                   'quat': np.array([-0.684, 0.726, 0.05, 0.008])}

        # --- Modified P2 to avoid singularity ---
        p2_pos = np.array([-0.159, 0.501, 0.415])
        p2_quat_mod = np.array([0.029, 0.98, 0.045, 0.033])
        p2_quat_mod /= np.linalg.norm(p2_quat_mod)

        # --- Flip P2 around tool Z axis by 180° to correct end-effector orientation ---
        r_p2 = R.from_quat(p2_quat_mod)
        r_flip = r_p2 * R.from_euler('z', 180, degrees=True)
        p2_quat_mod = r_flip.as_quat()

        self.p2_modified = {'pos': p2_pos, 'quat': p2_quat_mod}

        # --- PD Controller Gains ---
        self.KP_LINEAR = 1.8
        self.KP_ANGULAR = 1.2
        self.KD_LINEAR = 0.08
        self.KD_ANGULAR = 0.06

        # --- Limits and Tolerances ---
        self.POS_TOLERANCE = 0.15
        self.ORIENT_TOLERANCE = 0.15
        self.WAIT_TIME = 1.0
        self.MAX_LINEAR_VEL = 0.35
        self.MAX_ANGULAR_VEL = 0.55

        # --- ROS Interfaces ---
        self.twist_pub = self.create_publisher(Twist, '/delta_twist_cmds', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.timer = self.create_timer(self.dt, self.control_loop)

        # --- Internal State ---
        self.waypoints = None
        self.current_waypoint_index = 0
        self.state = 'INITIALIZING'
        self.wait_start_time = None
        self.all_goals_reached = False
        self.last_pos_error = np.zeros(3)
        self.last_orient_error = np.zeros(3)

        self.get_logger().info("✅ PD Controller initialized. Waiting for TF transform...")

        # Publish static P2 TF once
        self.broadcast_p2_static_tf()

    def broadcast_p2_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.REFERENCE_FRAME
        t.child_frame_id = "p2_modified"
        t.transform.translation.x = self.p2_modified['pos'][0]
        t.transform.translation.y = self.p2_modified['pos'][1]
        t.transform.translation.z = self.p2_modified['pos'][2]
        t.transform.rotation.x = self.p2_modified['quat'][0]
        t.transform.rotation.y = self.p2_modified['quat'][1]
        t.transform.rotation.z = self.p2_modified['quat'][2]
        t.transform.rotation.w = self.p2_modified['quat'][3]

        self.static_broadcaster.sendTransform(t)
        self.get_logger().info("📌 Static TF 'p2_modified' broadcasted.")

    # --- Main control loop ---
    def control_loop(self):
        if self.state == 'INITIALIZING':
            try:
                t = self.tf_buffer.lookup_transform(self.REFERENCE_FRAME, self.TARGET_FRAME, rclpy.time.Time())
                start_pose = {'pos': np.array([t.transform.translation.x,
                                               t.transform.translation.y,
                                               t.transform.translation.z]),
                              'quat': np.array([t.transform.rotation.x,
                                                t.transform.rotation.y,
                                                t.transform.rotation.z,
                                                t.transform.rotation.w])}

                # Define motion sequence
                major_waypoints = [self.p1, self.p3, start_pose, self.p2_modified]
                self.waypoints = self.generate_interpolated_path(start_pose, major_waypoints)

                self.get_logger().info(f"Path generated with {len(self.waypoints)} points. Starting motion...")
                self.state = 'MOVING'

                # Initialize errors
                target_waypoint = self.waypoints[self.current_waypoint_index]
                self.last_pos_error = target_waypoint['pos'] - start_pose['pos']
                q_error_init = R.from_quat(target_waypoint['quat']) * R.from_quat(start_pose['quat']).inv()
                self.last_orient_error = q_error_init.as_rotvec()

                return
            except TransformException:
                self.get_logger().info('Waiting for TF transform...', throttle_duration_sec=1.0)
                return

        if self.all_goals_reached:
            self.publish_zero_twist()
            if not self.timer.is_canceled():
                self.timer.cancel()
                self.get_logger().info("🎯 Task complete. All major waypoints visited.")
            return

        try:
            t = self.tf_buffer.lookup_transform(self.REFERENCE_FRAME, self.TARGET_FRAME, rclpy.time.Time())
            current_pos = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
            current_quat = np.array([t.transform.rotation.x, t.transform.rotation.y,
                                     t.transform.rotation.z, t.transform.rotation.w])
        except TransformException as ex:
            self.get_logger().error(f"Transform lost: {ex}")
            return

        target_waypoint = self.waypoints[self.current_waypoint_index]
        pos_error_vec = target_waypoint['pos'] - current_pos
        q_error = R.from_quat(target_waypoint['quat']) * R.from_quat(current_quat).inv()
        orient_error_vec = q_error.as_rotvec()

        pos_error_mag = np.linalg.norm(pos_error_vec)
        orient_error_mag = np.linalg.norm(orient_error_vec)
        is_at_goal = pos_error_mag < self.POS_TOLERANCE and orient_error_mag < self.ORIENT_TOLERANCE

        # --- MOVING STATE ---
        if self.state == 'MOVING':
            if is_at_goal:
                if target_waypoint['is_major']:
                    self.get_logger().info(f"✅ Reached Major Waypoint {target_waypoint['major_index'] + 1}")
                self.state = 'WAITING'
                self.wait_start_time = self.get_clock().now()
                self.publish_zero_twist()
            else:
                self.publish_velocity_command(pos_error_vec, orient_error_vec)

        # --- WAITING STATE ---
        elif self.state == 'WAITING':
            self.publish_zero_twist()
            elapsed_time = (self.get_clock().now() - self.wait_start_time).nanoseconds / 1e9
            wait_duration = self.WAIT_TIME if target_waypoint['is_major'] else 0.1

            if elapsed_time >= wait_duration:
                self.current_waypoint_index += 1
                if self.current_waypoint_index >= len(self.waypoints):
                    self.all_goals_reached = True
                else:
                    self.state = 'MOVING'
                    self.get_logger().info("⏩ Continuing path...")

    # --- Interpolation ---
    def generate_interpolated_path(self, start_pose, major_waypoints):
        full_path = []
        path_segments = [start_pose] + major_waypoints

        for i in range(len(path_segments) - 1):
            start_wp = path_segments[i]
            end_wp = path_segments[i + 1]

            key_rots = R.from_quat([start_wp['quat'], end_wp['quat']])
            slerp = Slerp([0, 1], key_rots)

            for step in range(1, self.NUM_INTER_STEPS + 1):
                interp_factor = step / self.NUM_INTER_STEPS
                inter_pos = start_wp['pos'] + interp_factor * (end_wp['pos'] - start_wp['pos'])
                inter_quat = slerp([interp_factor]).as_quat()[0]
                is_major_goal = (step == self.NUM_INTER_STEPS)

                full_path.append({
                    'pos': inter_pos,
                    'quat': inter_quat,
                    'is_major': is_major_goal,
                    'major_index': i
                })
        return full_path

    # --- PD Control ---
    def publish_velocity_command(self, pos_error, orient_error_vec):
        pos_error_deriv = (pos_error - self.last_pos_error) / self.dt
        orient_error_deriv = (orient_error_vec - self.last_orient_error) / self.dt

        linear_vel = self.KP_LINEAR * pos_error + self.KD_LINEAR * pos_error_deriv
        angular_vel = self.KP_ANGULAR * orient_error_vec + self.KD_ANGULAR * orient_error_deriv

        if np.linalg.norm(linear_vel) > self.MAX_LINEAR_VEL:
            linear_vel = (linear_vel / np.linalg.norm(linear_vel)) * self.MAX_LINEAR_VEL
        if np.linalg.norm(angular_vel) > self.MAX_ANGULAR_VEL:
            angular_vel = (angular_vel / np.linalg.norm(angular_vel)) * self.MAX_ANGULAR_VEL

        twist_msg = Twist()
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = linear_vel
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = angular_vel
        self.twist_pub.publish(twist_msg)

        self.last_pos_error = pos_error
        self.last_orient_error = orient_error_vec

    def publish_zero_twist(self):
        self.twist_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = ArmWaypointController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Keyboard interrupt. Shutting down...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
