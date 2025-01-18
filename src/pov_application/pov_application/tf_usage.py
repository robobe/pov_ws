#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from tf_transformations import euler_from_quaternion

class TfSubscriberNode(Node):
    def __init__(self):
        super().__init__('tf_subscriber_node')

        # Create a buffer and TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Set a timer to query TF tree at 10 Hz
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        try:
            # Get the current time
            now = self.get_clock().now()

            # Lookup transform between frames
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'world',  # Target frame
                'camera',   # Source frame
                Time(seconds=0),    # Time (0 means latest available transform)
                timeout=rclpy.duration.Duration(seconds=1.0)  # Timeout for lookup
            )

            # Log the transformation
            self.get_logger().info(f"Transform from 'world' to 'camera_optical':")
            self.get_logger().info(f"Translation: x={transform.transform.translation.x}, "
                                   f"y={transform.transform.translation.y}, "
                                   f"z={transform.transform.translation.z}")
            q = transform.transform.rotation
            r,p,y = euler_from_quaternion([q.x, q.y, q.z, q.w])
            import math
            # r,p,y = map(lambda x: math.degrees(x), rpy)
            r,p,y = math.degrees(r),math.degrees(p),math.degrees(y)
            self.get_logger().info(f"{r=},{p=},{y=}")
            # self.get_logger().info(f"Rotation: x={transform.transform.rotation.x}, "
            #                        f"y={transform.transform.rotation.y}, "
            #                        f"z={transform.transform.rotation.z}, "
            #                        f"w={transform.transform.rotation.w}")
        except Exception as e:
            # Handle exceptions (e.g., transform not available)
            self.get_logger().error(f"Could not get transform: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TfSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
