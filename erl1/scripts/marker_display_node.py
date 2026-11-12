#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from aruco_opencv_msgs.msg import MarkerPose
from aruco_opencv_msgs.msg import ArucoDetection
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo
from custom_msgs.msg import Box
from cv_bridge import CvBridge
from std_msgs.msg import Int32
from math import sqrt
import numpy as np
import threading
import sys
import math
import cv2

class RobotController(Node):
   

    def __init__(self):

        super().__init__('robot_controller')

        # Setting Parameters 
        self.declare_parameter('aruco_topic', 'aruco_detections')
        self.declare_parameter('camera_info_topic', 'camera/camera_info')
        self.declare_parameter('target_reached_topic', 'target_reached')
        self.declare_parameter('image_topic', 'camera/image')
        self.declare_parameter('marker_image_topic', 'marker_image')
        self.declare_parameter('custom_msg_topic', 'custom_msg')
        self.declare_parameter('marker_size', 0.2)

        def param(name):
            return self.get_parameter(name).value

        # --- 2. DECLARING MESSAGES ---
        # Instead of just declaring, we create the empty object instance here.
        self.marker_info = ArucoDetection()
        self.image_info = CameraInfo()
        self.raw_image  = CompressedImage()
        self.custom_msg = Box()
        self.bridge = CvBridge()


        # --- 3. DECLARING VARIABLES ---
        self.marker_id = 0.0
        self.pose = 0.0
        self.orientation = 0.0
        self.k_matrix = None
        self.notify = False
        self.marker_size = param('marker_size')

        # 2D position of the marker in image frame
        self.x_px = 0.0
        self.y_px = 0.0
        self.radius = 0.0
        self.z_depth = 0.0

        #3D position of the marker in camera frame
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.first_marker = []  
        self.has_marker = False   
        self.marker_array = []


        self.subscription = self.create_subscription(ArucoDetection, param('aruco_topic'), self.aruco_callback, 10)
        self.subscription_intrinsic = self.create_subscription(CameraInfo, param('camera_info_topic'),self.intrinsic_callback, 10)
        self.subscription_marker_info = self.create_subscription(Int32, param('target_reached_topic'),self.target_reached_callback, 10)
        self.subscription_image = self.create_subscription(CompressedImage, param('image_topic'),self.image_callback, 10)

        self.publisher_marker = self.create_publisher(CompressedImage, param('marker_image_topic'), 10)
        self.publisher_msg = self.create_publisher(Box, param('custom_msg_topic'), 10)

        self.get_logger().info('Robot Controller Node has been started.')


    def aruco_callback(self, msg):

        self.marker_info = msg

        #Safety check
        # If no marker detected (notified from a Boolean topic)
        # No marker info
        # No intrinsic matrix received
        if (len(self.marker_info.markers) == 0) or self.k_matrix is None:
            self.radius = 0.0
            self.x_px = 0.0
            self.y_px = 0.0
            self.z_depth = 0.0
            return

        # Need this because we compare id to make sure we are at the target
        self.first_marker = self.marker_info.markers[0]
        self.marker_id = self.first_marker.marker_id 
        #self.get_logger().info(f"I see {self.marker_id}")
        
        if self.marker_id in self.marker_array or self.marker_id == 0 or( self.marker_array and self.marker_id != self.marker_array[-1] + 1):
            #self.get_logger().info(f"I ignore {self.marker_id}")
            self.has_marker = False
            return
        else:
            self.get_logger().info(f"{self.marker_array}")
            self.has_marker = True     

            # 3d position of the marker in camera frame               
            self.x = self.first_marker.pose.position.x
            self.y = self.first_marker.pose.position.y
            self.z = self.first_marker.pose.position.z
            self.orientation = self.first_marker.pose.orientation               # x, y, z, w
            # self.get_logger().info(f'Marker information received.')

            # 2D position of the marker in image frame
            self.x_px = self.k_matrix[0,0]*self.x/self.z + self.k_matrix[0,2]
            self.y_px = self.k_matrix[1,1]*self.y/self.z + self.k_matrix[1,2]
            self.radius = (self.marker_size*sqrt(2)/2)*self.k_matrix[0,0]/self.z
            self.z_depth = self.z
        
            # Info needed for second part of assignment 
            self.custom_msg.id = int(self.marker_id)
            self.custom_msg.cx = int(self.x_px)
            self.custom_msg.cy = int(self.y_px)
            self.custom_msg.d = float(self.z_depth)

            self.publisher_msg.publish(self.custom_msg)
            # self.get_logger().info(f'Published custom message')
            

    def target_reached_callback(self, Int32):
        if Int32.data == self.marker_id:
            self.notify = True
            self.marker_array.append(self.marker_id)
            self.get_logger().info(f'marker appended')
            self.get_logger().info(f"{self.marker_array}")
        else:
            self.notify = False

    def intrinsic_callback(self, msg):

        # Converting array to 3x3 matrix
        self.image_info = msg
        self.k_matrix =np.array(self.image_info.k).reshape(3,3)

        # self.get_logger().info(f'Intrinsic matrix for camera')
        


    def image_callback(self, msg):

        # self.get_logger().info('Image received.')

        # convert ROS Image message to OpenCV image
        #img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Publishing on the marker_image topic
        # Ever time the robot is near the marker then we will publish the red circle and the marker id on the image
        # Will be visualized in rviz2
        marker_cx = int(self.x_px)
        marker_cy = int(self.y_px)
        r = int(self.radius)
                
        if self.notify==False and self.has_marker:   
            #Drawing a circle around the marker and displaying the marker id on the image
            cv2.circle(img, (marker_cx,marker_cy ), r, (0, 0, 255), 2)
            cv2.putText(img,f"id {self.marker_id}",(marker_cx - r, marker_cy - r - 10),cv2.FONT_HERSHEY_SIMPLEX,1.0,(0, 0, 255),3,)

        #out_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        out_msg = self.bridge.cv2_to_compressed_imgmsg(img, dst_format='jpg')
        out_msg.header = msg.header   
            
        # Publish the message 
        self.publisher_marker.publish(out_msg)
        self.get_logger().info(f'Reached Target and drawn circle')
        

    

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()

    try: 
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()   