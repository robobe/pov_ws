#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import math # Math library
# from transforms3d
from tf_transformations import euler_from_quaternion

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        # Subscription to the image topic
        self.subscription = self.create_subscription(
            Image,
            '/gz_camera',  # Replace with your topic name
            self.image_callback,
            10  # QoS profile depth
        )

        self.camera_info = self.create_subscription(
            CameraInfo,
            '/gz_camera_info',  # Replace with your topic name
            self.image_callback_info,
            10  # QoS profile depth
        )
        self.camera_matrix = None
        self.dist_coeffs = None
        self.bridge = CvBridge()
        self.get_logger().info("Image subscriber node started. Listening for images on /camera/image")

    def aruco_pose(self, image):
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict,
            parameters=arucoParams)


        cv2.aruco.drawDetectedMarkers(image, corners, ids)

        aruco_marker_side_length = 1.0
        # camera_matrix = np.array([
        #     [800, 0, 640],  # Focal lengths (fx, fy) and principal point (cx)
        #     [0, 800, 360],
        #     [0, 0, 1]
        # ], dtype=float)

        # # Predefined distortion coefficients (assuming minor distortion)
        # dist_coeffs = np.array([0, 0, 0, 0, 0], dtype=float)

        # Get the rotation and translation vectors
        rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
        corners,
        aruco_marker_side_length,
        self.camera_matrix,
        self.dist_coeffs)

        for i, marker_id in enumerate(ids):
            
            # Store the translation (i.e. position) information
            transform_translation_x = tvecs[i][0][0]
            transform_translation_y = tvecs[i][0][1]
            transform_translation_z = tvecs[i][0][2]

            # Store the rotation information
            rotation_matrix = np.eye(4)
            rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
            r = R.from_matrix(rotation_matrix[0:3, 0:3])
            quat = r.as_quat()   
                
            # Quaternion format     
            transform_rotation_x = quat[0] 
            transform_rotation_y = quat[1] 
            transform_rotation_z = quat[2] 
            transform_rotation_w = quat[3] 
                
            # Euler angle format in radians
            roll_x, pitch_y, yaw_z = euler_from_quaternion([transform_rotation_x, 
                                                            transform_rotation_y, 
                                                            transform_rotation_z, 
                                                            transform_rotation_w])
                
            roll_x = math.degrees(roll_x)
            pitch_y = math.degrees(pitch_y)
            yaw_z = math.degrees(yaw_z)
            print("transform_translation_x: {}".format(transform_translation_x))
            print("transform_translation_y: {}".format(transform_translation_y))
            print("transform_translation_z: {}".format(transform_translation_z))
            print("roll_x: {}".format(roll_x))
            print("pitch_y: {}".format(pitch_y))
            print("yaw_z: {}".format(yaw_z))
            print()
                
            # Draw the axes on the marker
            cv2.aruco.drawAxis(image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.05)
            
        # Display the resulting frame
        cv2.imshow('frame',image)
        cv2.waitKey(0)


    def image_callback_info(self, msg):
        # Log or process the camera info
        self.get_logger().info(f"Received Camera Info: {msg}")
        self.get_logger().info(f"Camera Width: {msg.width}, Height: {msg.height}")
        self.get_logger().info(f"Distortion Model: {msg.distortion_model}")
        self.get_logger().info(f"K (Intrinsic Matrix): {msg.k}")
        self.get_logger().info(f"D (Distortion Coefficients): {msg.d}")

        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

        self.destroy_subscription(self.camera_info)


    def image_callback(self, msg):
        if self.camera_matrix is None:
            self.get_logger().warn("Missing camera ino")
            return
        
        try:
            # Convert ROS Image message to OpenCV image
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.aruco_pose(image)
            # Display the image using OpenCV
            cv2.imshow("Camera Image", image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    # Create the node
    node = ImageSubscriber()
    
    try:
        # Keep the node running
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
