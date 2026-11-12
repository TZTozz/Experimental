#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from custom_msgs.msg import Box

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

class TreasureFinder(Node):
    def __init__(self):
        super().__init__('treasure_finder')

        # Declare and read parameters matching treasure_finder.yaml
        self.declare_parameter('velocity_topic', 'cmd_vel')
        self.declare_parameter('velocity_info_topic', 'velocity_info')
        self.declare_parameter('reached_topic', 'target_reached')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('marker_info_topic', 'marker_info')

        vel_topic = self.get_parameter('velocity_topic').value
        vel_info_topic = self.get_parameter('velocity_info_topic').value
        reached_topic = self.get_parameter('reached_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        marker_info_topic = self.get_parameter('marker_info_topic').value

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, vel_topic, 10)
        self.reached_pub = self.create_publisher(Int32, reached_topic, 10)

        # Subscribers
        self.vel_info_sub = self.create_subscription(
            Twist, vel_info_topic, self.velocity_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10)
        self.target_sub = self.create_subscription(
            Int32, reached_topic, self.target_callback, 10)
        self.marker_sub = self.create_subscription(
            Box, marker_info_topic, self.marker_info_callback, 10)

        # State tracking
        self.processed_markers = set()
        self.current_seen_id = -1
        self.last_valid_vel_time = None
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.state = 'SERVOING'  # States: SERVOING, ROTATING, STOPPED
        self.rotation_error_threshold = 0.04

        # Control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Treasure Finder started in SERVOING mode.')

    def marker_info_callback(self, msg):
        self.current_seen_id = msg.id

    def velocity_callback(self, msg):
        if self.state != 'SERVOING':
            return

        # Ignore visual commands if looking at an already seen marker
        if self.current_seen_id in self.processed_markers:
            return

        self.last_valid_vel_time = self.get_clock().now()
        self.cmd_pub.publish(msg)

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        # Assignment quaternion-to-yaw formula
        self.current_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y ** 2 + q.z ** 2)
        )

    def target_callback(self, msg):
        marker_id = msg.data

        # Ignore reset messages (0) or already processed markers
        if marker_id <= 0 or self.state == 'STOPPED' or marker_id in self.processed_markers:
            return

        self.processed_markers.add(marker_id)

        if marker_id == 4:
            self.state = 'STOPPED'
            self.stop_robot()
            self.get_logger().info('Marker 4 reached! Treasure found! Hunt is over.')
            return

        self.get_logger().info(f'Marker {marker_id} reached! Executing rotation instruction.')

        if marker_id == 1:
            self.target_yaw = normalize_angle(self.current_yaw - (math.pi / 2.0))  # Turn right 90 deg
            self.state = 'ROTATING'
        elif marker_id == 2:
            self.target_yaw = normalize_angle(self.current_yaw + math.pi)          # Rotate 180 deg
            self.state = 'ROTATING'
        elif marker_id == 3:
            self.target_yaw = normalize_angle(self.current_yaw + (math.pi / 2.0))  # Turn left 90 deg
            self.state = 'ROTATING'

    def control_loop(self):
        if self.state == 'STOPPED':
            self.stop_robot()
            return

        if self.state == 'ROTATING':
            err = normalize_angle(self.target_yaw - self.current_yaw)
            twist = Twist()

            if abs(err) > self.rotation_error_threshold:
                twist.angular.z = max(min(0.5 * err, 0.02), -0.02)
                self.cmd_pub.publish(twist)
            else:
                self.get_logger().info('Rotation complete. Resetting target_reached and moving ahead.')
                self.stop_robot()
                reset_msg = Int32()
                reset_msg.data = 0
                self.reached_pub.publish(reset_msg)

                self.last_valid_vel_time = None
                self.state = 'SERVOING'

        elif self.state == 'SERVOING':
            # Drive straight ahead if no new marker is currently being tracked by velocity_node
            now = self.get_clock().now()
            no_recent_servo = (
                self.last_valid_vel_time is None or
                (now - self.last_valid_vel_time).nanoseconds > 500_000_000
            )
            if no_recent_servo:
                twist = Twist()
                twist.linear.x = 0.02
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TreasureFinder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()