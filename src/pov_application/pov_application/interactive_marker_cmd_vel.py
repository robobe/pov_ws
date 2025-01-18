#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker


class MinimalInteractiveMarkerNode(Node):
    def __init__(self):
        super().__init__('minimal_interactive_marker')
        self.get_logger().info("Starting Interactive Marker Server...")

        # Initialize the interactive marker server
        self.server = InteractiveMarkerServer(self, "minimal_marker_server")

        # Create and insert an interactive marker
        self.create_interactive_marker()

        # Apply changes to the server
        self.server.applyChanges()

    def create_interactive_marker(self):
        # Define the interactive marker
        marker = InteractiveMarker()
        marker.header.frame_id = "map"  # Use a valid frame in your TF
        marker.name = "simple_marker"
        marker.description = "A minimal interactive marker"
        marker.pose.position.x = 1.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        # Create a basic cube marker
        cube = Marker()
        cube.type = Marker.CUBE
        cube.scale.x = 0.2
        cube.scale.y = 0.2
        cube.scale.z = 0.2
        cube.color.r = 0.0
        cube.color.g = 1.0
        cube.color.b = 0.0
        cube.color.a = 1.0

        # Add the cube to the interactive marker control
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(cube)
        marker.controls.append(control)

        # Add interactive control for moving along the X-axis
        move_control = InteractiveMarkerControl()
        move_control.name = "move_x"
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(move_control)

        # Insert the marker into the server
        self.server.insert(marker, self.process_feedback)

    def process_feedback(self, feedback):
        self.get_logger().info(f"Marker {feedback.marker_name} moved to: {feedback.pose.position}")


def main(args=None):
    rclpy.init(args=args)
    node = MinimalInteractiveMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Interactive Marker Server.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
