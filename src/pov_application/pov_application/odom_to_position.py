#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Pose, Point, Quaternion
from std_msgs.msg import Header
from rclpy.time import Time
from rclpy.clock import Clock
import numpy as np
import math
from tf_transformations import (
    euler_from_quaternion,
    rotation_matrix,
    quaternion_conjugate,
    quaternion_multiply
)

"""
EK2_ENABLE = 1
AHRS_EKF_TYPE = 2 (ekf2)
VISIO_TYPE = 1 (mavlink)
COMPASS_USE = COMPASS_USE2 = COMPASS_USE3 = 0

"""

class GazeboToMavrosOdometry(Node):
    def __init__(self):
        super().__init__("gazebo_to_mavros_odometry")
        self.clock = Clock()
        # Declare parameters
        # Publisher for MAVROS odometry
        self._pub = self.create_publisher(PoseWithCovarianceStamped, "/mavros/vision_pose/pose_cov", 10)
        # Subscriber to Gazebo odometry
        self.create_subscription(Odometry, "/sim_odom", self.odom_callback, 10)



    def enu2ned_matrix(self):
        r = rotation_matrix(np.pi, [1,0,0])
        y = rotation_matrix(np.pi/2, [0,0,1])
        m = np.dot(y,r)
        return m
    
    
    def odom_callback(self, msg: Odometry):
        """
        Callback to process Gazebo odometry and publish to MAVROS.
        """


        p_msg = PoseWithCovarianceStamped()
        p_msg.header = Header()
        p_msg.header.frame_id = "map"
        p_msg.header.stamp = self.clock.now().to_msg()

        # p_msg.pose.pose.position = Point(x=0.0,y=0.0,z=0.0)
        # p_msg.pose.pose.orientation = Quaternion(x=0.0,y=0.0,z=0.0,w=1.0)
        p_msg.pose.pose = msg.pose.pose
        p_msg.pose.covariance = [0.0]*36
        POS_COV = 1.0
        ANGLE_COV = 1.0

        for i in [0,7,14]:
            p_msg.pose.covariance[i] = POS_COV

        for i in [21,28,35]:
            p_msg.pose.covariance[i] = ANGLE_COV

        self._pub.publish(p_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboToMavrosOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
