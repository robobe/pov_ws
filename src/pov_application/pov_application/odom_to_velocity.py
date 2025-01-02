#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped, Vector3, TwistWithCovarianceStamped, TwistWithCovariance
from std_msgs.msg import Header
from rclpy.time import Time
from rclpy.clock import Clock
import numpy as np
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
        self.vector_pub = self.create_publisher(Vector3Stamped, "/mavros/vision_speed/speed_vector", 10)
        self.twist_cov_pub = self.create_publisher(TwistWithCovarianceStamped, "/mavros/vision_speed/speed_twist_cov", 10)
        # Subscriber to Gazebo odometry
        self.create_subscription(Odometry, "/sim_odom", self.odom_callback, 10)



    def enu2ned_matrix(self):
        r = rotation_matrix(np.pi, [1,0,0])
        y = rotation_matrix(np.pi/2, [0,0,1])
        m = np.dot(y,r)
        return m
    
    def convert_velocity_to_global_matrix_q(self, odom_msg: Odometry):

        v_body = np.array([odom_msg.twist.twist.linear.x,  # v_x
                        odom_msg.twist.twist.linear.y,
                        odom_msg.twist.twist.linear.z,
                        1]) # v_y
        speed_enu_vector = odom_msg.pose.pose.orientation


        quat = [speed_enu_vector.x, speed_enu_vector.y, speed_enu_vector.z, speed_enu_vector.w]
        q_conjugate = quaternion_conjugate(quat)

        speed_enu_vector = quaternion_multiply(quaternion_multiply(quat, v_body), q_conjugate)
        # # extract vector from result
        # v = np.array(q[:3])
        # v= np.array([v[0], v[1], v[2], 1])
        # enu2ned = self.enu2ned_matrix()
        # v_ned = np.dot(enu2ned, v.T)
        # return v_ned[0], v_ned[1]
        return speed_enu_vector[0], speed_enu_vector[1], speed_enu_vector[2]

    def convert_velocity_to_global_matrix(self, odom_msg: Odometry):
        # Extract velocities in the body frame
        v_body = np.array([odom_msg.twist.twist.linear.x,  # v_x
                        odom_msg.twist.twist.linear.y,
                        odom_msg.twist.twist.linear.z,
                        1]) # v_y
    
        # Extract orientation quaternion
        q = odom_msg.pose.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
    
        # Convert quaternion to Euler angles
        _, _, yaw = euler_from_quaternion(quaternion)
        R = rotation_matrix(yaw, [0, 0, 1])

       
        
        # Perform the transformation
        v_global = np.dot(R, v_body)
        
        return v_global[0], v_global[1]  # v_x_global, v_y_global
    
    def odom_callback(self, msg: Odometry):
        """
        Callback to process Gazebo odometry and publish to MAVROS.
        """
        vx, vy, vz = self.convert_velocity_to_global_matrix_q(msg)

        # v_msg = Vector3Stamped()
        # v_msg.header = Header()
        # v_msg.header.frame_id = "odom"
        # v_msg.header.stamp = Time().to_msg()

        # v_msg.vector = Vector3(x=vx, y=vy, z=vz)
        # self.vector_pub.publish(v_msg)

        t_msg = TwistWithCovarianceStamped()
        t_msg.header = Header()
        t_msg.header.frame_id = "odom"
        t_msg.header.stamp = self.clock.now().to_msg()

        t_msg.twist = TwistWithCovariance()
        t_msg.twist.twist.linear = Vector3(x=vx, y=vy, z=vz)

        VEL_COV = 1
        t_msg.twist.covariance = [0.0]*36
        t_msg.twist.covariance[0] = VEL_COV
        t_msg.twist.covariance[7] = VEL_COV
        t_msg.twist.covariance[14] = VEL_COV

        self.twist_cov_pub.publish(t_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboToMavrosOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
