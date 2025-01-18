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
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_broadcaster import   TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from tf_transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        self.camera_to_camera_optical()
        self.br = TransformBroadcaster(self)
        self.publish_aruco_tf()
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

        ARUCO_MARKER_SIDE_LENGTH = 1
        # camera_matrix = np.array([
        #     [800, 0, 640],  # Focal lengths (fx, fy) and principal point (cx)
        #     [0, 800, 360],
        #     [0, 0, 1]
        # ], dtype=float)

        # # Predefined distortion coefficients (assuming minor distortion)
        # dist_coeffs = np.array([0, 0, 0, 0, 0], dtype=float)

        # Get the rotation and translation vectors

        # --
        # pose (position and orientation) of a single ArUco marker in a 3D space relative to the camera

        # Corners: detect aruco
        #marker length: The physical length of the side of the ArUco marker in the real world.
        # ---
        # rvec: rotation vector
        # tvec: translation vector
        rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
        corners,
        ARUCO_MARKER_SIDE_LENGTH,
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
            self.publish_aruco_camera_tf(
                transform_translation_x,
                transform_translation_y,
                transform_translation_z,
                quat
            )
            print("transform_translation_x: {}".format(transform_translation_x))
            print("transform_translation_y: {}".format(transform_translation_y))
            print("transform_translation_z: {}".format(transform_translation_z))
            print("roll_x: {}".format(roll_x))
            print("pitch_y: {}".format(pitch_y))
            print("yaw_z: {}".format(yaw_z))
            print()
                
            # Draw the axes on the marker
            cv2.aruco.drawAxis(image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.5)
            
        # Display the resulting frame
        # cv2.imshow('frame',image)
        # cv2.waitKey(0)


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

    def publish_aruco_camera_tf(self, x, y, z, q):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_optical'  # Parent frame
        t.child_frame_id = 'aruco'  # Child frame

        # Set dynamic translation (e.g., a sinusoidal movement)
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        # Set dynamic rotation (e.g., continuous rotation around Z-axis)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Publish the transform
        self.br.sendTransform(t)

    def camera_to_camera_optical(self):
        self.static_broadcaster = StaticTransformBroadcaster(self)
        pi = math.pi
        y = rotation_matrix(-pi/2,[0,0,1])
        r = rotation_matrix(-pi/2,[1,0,0])

        rot = y@r
        q = quaternion_from_matrix(rot)

        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'camera'
        static_transform.child_frame_id = 'camera_optical'

        # Translation
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0

        # Rotation (Quaternion)
        static_transform.transform.rotation.x = q[0]
        static_transform.transform.rotation.y = q[1]
        static_transform.transform.rotation.z = q[2]
        static_transform.transform.rotation.w = q[3]

        # Publish the static transform
        self.static_broadcaster.sendTransform(static_transform)
        print('==============================================================')

    def publish_aruco_tf(self):
        
        self.static_broadcaster = StaticTransformBroadcaster(self)
        # Define the static transform
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'aruco'
        static_transform.child_frame_id = 'world'

        # Translation
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = -0.005

        # Rotation (Quaternion)
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0

        # Publish the static transform
        self.static_broadcaster.sendTransform(static_transform)

        self.get_logger().info('Static transform published!')

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
