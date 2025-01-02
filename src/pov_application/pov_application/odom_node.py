#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from tf_transformations import (
    euler_from_quaternion,
    rotation_matrix,
    quaternion_conjugate,
    quaternion_multiply
)


class GazeboToMavrosOdometry(Node):
    def __init__(self):
        super().__init__("gazebo_to_mavros_odometry")

        # Declare parameters
        self.declare_parameter("gazebo_odom_topic", "/sim_odom")
        self.declare_parameter("mavros_odom_topic", "/mavros/odometry/out")

        # Get parameters
        gazebo_odom_topic = self.get_parameter("gazebo_odom_topic").get_parameter_value().string_value
        mavros_odom_topic = self.get_parameter("mavros_odom_topic").get_parameter_value().string_value

        # Publisher for MAVROS odometry
        self.mavros_odom_pub = self.create_publisher(Odometry, mavros_odom_topic, 10)

        # Subscriber to Gazebo odometry
        self.create_subscription(Odometry, gazebo_odom_topic, self.odom_callback, 10)

        self.get_logger().info(f"Subscribed to {gazebo_odom_topic}, publishing to {mavros_odom_topic}")


    def enu2ned_matrix(self):
        r = rotation_matrix(np.pi, [1,0,0])
        y = rotation_matrix(np.pi/2, [0,0,1])
        m = np.dot(y,r)
        return m
    
    def convert_velocity_to_global_matrix_q(self, odom_msg: Odometry):
        v_body = np.array([odom_msg.twist.twist.linear.x,  # v_x
                        odom_msg.twist.twist.linear.y,
                        odom_msg.twist.twist.linear.z,
                        0]) # v_y
        q = odom_msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        q_conjugate = quaternion_conjugate(quat)

        q = quaternion_multiply(quaternion_multiply(quat, v_body), q_conjugate)
        # extract vector from result
        v = np.array(q[:3])
        v= np.array([v[0], v[1], v[2], 1])
        enu2ned = self.enu2ned_matrix()
        v_ned = np.dot(enu2ned, v.T)
        return v_ned[0], v_ned[1]

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
        vx, vy = self.convert_velocity_to_global_matrix_q(msg)
        print(vx, vy)
        # Publish the modified odometry message
        # self.mavros_odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboToMavrosOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
