#!/usr/bin/env python3


import rclpy
from rclpy.executors import MultiThreadedExecutor
from pov_application.mavlink import Sender , Receiver
from pov_application.pov_node import POVNode
from rclpy.qos import (
    qos_profile_sensor_data,
    qos_profile_services_default,
    qos_profile_system_default,
)
import time
from nav_msgs.msg import Odometry
import numpy as np
from tf_transformations import (
    euler_from_quaternion,
    rotation_matrix,
    quaternion_conjugate,
    quaternion_multiply
)

class MyNode(POVNode):
    def __init__(self):
        node_name="mav_node"
        super().__init__(node_name)
        self.__time_sync_delta = 0
        self.__mav_sender = Sender()
        self.__mav_receiver = Receiver()
        self.__mav_sender.run()
        self.__mav_receiver.run()
        self.__mav_receiver.register_handler("HEARTBEAT", self.__heartbeat_handler)
        self.__mav_receiver.register_handler("SYSTEM_TIME", self.__system_time_handler)
        self.__timer = self.create_timer(1.0, self.__timer_handler)
        self.__init_subscribers()
        self.get_logger().info("Hello ROS2")

    def __init_subscribers(self):
        self.sub_odom = self.create_subscription(
            Odometry,
            "/sim_odom",
            self.odom_handler,
            qos_profile=qos_profile_system_default,
            callback_group=self.g,
        )


    def enu2ned_matrix(self):
        r = rotation_matrix(np.pi, [1,0,0])
        y = rotation_matrix(np.pi/2, [0,0,1])
        m = np.dot(y,r)
        return m
    
    def flu2frd_matrix(self):
        return np.array([
            [1,0,0,0],
            [0,-1,0,0],
            [0,0,-1,np.pi/2],
            [0,0,0,1]
        ])
    
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
        return v_ned 
    
    
    def odom_handler(self, msg: Odometry):
        v_ned = self.convert_velocity_to_global_matrix_q(msg)
        # print(v_ned)
        # msec = int(time.time()*1e3 + self.__time_sync_delta)
        # usec = int(msec * 1e3)
        usec = int(time.time()*1e6)
        self.__mav_sender.send_vision_speed_estimate(usec, v_ned[0], v_ned[1], v_ned[2])

        enu2ned = self.enu2ned_matrix()
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        pos_ned = np.dot(enu2ned, np.array([p.x, p.y, p.z, 1]).T)
        e = euler_from_quaternion([o.x, o.y, o.z, o.w])
        flu2frd_matrix = self.flu2frd_matrix()
        orientation_ned = np.dot(flu2frd_matrix, np.array([e[0], e[1], e[2], 1]).T)
        print(pos_ned)

        self.__mav_sender.send_vision_position_estimate(
            usec,
            pos_ned[0],
            pos_ned[1],
            pos_ned[2],
            orientation_ned[0],
            orientation_ned[1],
            orientation_ned[2],
        )

    #endregion
    def __timer_handler(self):
        pass
        # self.get_logger().info("Timer handler")

    def __heartbeat_handler(self, mav_msg):
        self.get_logger().info(f"{mav_msg.type}")
        
    def __system_time_handler(self, mav_msg):
        # self.get_logger().info(f"{mav_msg}")
        t = time.time()*1e3
        self.__time_sync_delta = mav_msg.time_boot_ms - t

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    executer = MultiThreadedExecutor()
    executer.add_node(node)
    executer.spin()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()