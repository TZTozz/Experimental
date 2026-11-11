#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class TreasureFinder(Node):
    def __init__(self):
        super().__init__('treasure_finder')

        # Parameters
        self.declare_parameter('reached_topic', 'marker_reached')
        self.declare_parameter('velocity_topic', 'cmd_vel')

        def param(name):
                    return self.get_parameter(name).value
        
        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(Twist, param("velocity_topic"), 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.marker_sub = self.create_subscription(Int32, param("reached_topic"), self.marker_callback, 10)
        
        # State tracking
        self.processed_markers = set()
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.state = 'IDLE' # States: IDLE, ROTATING, MOVING_FORWARD, STOPPED
        self.rotation_error_threshold = 0.005
        
        # Control Loop running at 10Hz 
        self.timer = self.create_timer(0.1, self.control_loop)
        
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y**2 + q.z**2))
        
    def marker_callback(self, msg):
        marker_id = msg.data
        
        # Ignore already processed markers or if the hunt is finished
        if marker_id in self.processed_markers or self.state == 'STOPPED':
            return
            
        self.get_logger().info(f'Processing new ArUco ID: {marker_id}')
        self.processed_markers.add(marker_id)
        
        # Map IDs to target angles
        if marker_id == 1:
            self.target_yaw = self.current_yaw - (math.pi / 2.0) # Turn right 90°
            self.state = 'ROTATING'
        elif marker_id == 2:
            self.target_yaw = self.current_yaw + math.pi         # Rotate 180°
            self.state = 'ROTATING'
        elif marker_id == 3:
            self.target_yaw = self.current_yaw + (math.pi / 2.0) # Turn left 90°
            self.state = 'ROTATING'
        elif marker_id == 4:
            self.state = 'STOPPED'
            self.stop_robot()
            self.get_logger().info('Treasure found! Hunt is over.')
            
    def control_loop(self):
        if self.state == 'ROTATING':
            # Calculate normalized error in [-pi, pi]
            err = math.atan2(math.sin(self.target_yaw - self.current_yaw), 
                             math.cos(self.target_yaw - self.current_yaw))
            
            twist = Twist()
            if abs(err) > self.rotation_error_threshold:
                # Proportional controller (P-gain = 1.5)
                twist.angular.z = 3 * err 
                # Clamp max rotation speed
                twist.angular.z = max(min(twist.angular.z, 1.0), -1.0)
                self.cmd_pub.publish(twist)
            else:
                self.get_logger().info(f'error at {abs(err)}')
                self.get_logger().info('Rotation complete, moving forward.')
                self.stop_robot()
                self.state = 'MOVING_FORWARD'
                
        elif self.state == 'MOVING_FORWARD':
            # Push the robot forward so the visual servoing node can take over
            twist = Twist()
            twist.linear.x = 0.3
            self.cmd_pub.publish(twist)
            
            # Immediately reset to IDLE (because "marker_reached = false")
            self.state = 'IDLE'

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TreasureFinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()