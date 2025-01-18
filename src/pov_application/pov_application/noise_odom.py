#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import random


class NoiseOdometry(Node):
    def __init__(self):
        super().__init__("noise_odom")


        # Publisher for MAVROS odometry
        self.pub_noise_odom = self.create_publisher(Odometry, "noise_odom", 10)

        # Subscriber to Gazebo odometry
        self.create_subscription(Odometry, "sim_odom", self.odom_callback, 10)

    def odom_callback(self, msg: Odometry):
        rand_float = random.uniform(-0.5, 0.5)
        msg.pose.pose.position.x = msg.pose.pose.position.x + rand_float
        self.pub_noise_odom.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NoiseOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()