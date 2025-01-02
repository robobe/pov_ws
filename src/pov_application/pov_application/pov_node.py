from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup


class POVNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name, namespace="pov")
        self.g = ReentrantCallbackGroup()