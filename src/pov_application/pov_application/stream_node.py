#!/usr/bin/env python3

# very importent
# https://github.com/ros2/rclpy/issues/1149
"""
gst-launch-1.0 \
        videotestsrc \
        ! video/x-raw,width=800,height=600 \
        ! videoconvert \
        ! video/x-raw,format=I420 \
        ! videoconvert \
        ! x264enc name=codec bitrate=500  \
        ! rtph264pay  \
        ! udpsink host=10.0.0.2 port=5000 sync=true
        """
import traceback

import gi
import numpy as np
import cv2
#
# Stream Image message using gstreamer
#
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from rcl_interfaces.msg import (
    IntegerRange,
    ParameterDescriptor,
    ParameterType,
    SetParametersResult,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

# from rome_common import RomeNode

gi.require_version("Gst", "1.0")
from threading import Thread
from typing import List

from gi.repository import GLib, Gst

CAMERA_TOPIC = "/gz_camera"


# region gst
# # Initializes Gstreamer, it's variables, paths
Gst.init(None)


def on_message(bus: Gst.Bus, message: Gst.Message, loop: GLib.MainLoop):
    mtype = message.type
    if mtype == Gst.MessageType.EOS:
        print("End of stream")
        loop.quit()

    elif mtype == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        print(err, debug)
        loop.quit()

    elif mtype == Gst.MessageType.WARNING:
        err, debug = message.parse_warning()
        print(err, debug)

    return True


app_source = None
pipeline = None


def change_bitrate(rate):
    def execute(rate):
        element = pipeline.get_by_name("codec")
        element.set_property("bitrate", rate)

    GLib.idle_add(execute, rate)


def change_codec_prop(prop, value):
    def execute():
        element = pipeline.get_by_name("codec")
        print(prop, value)
        element.set_property(prop, value)

    GLib.idle_add(execute)


def gst_runner(bitrate):
    # 1 (downstream): Drop the oldest buffer.
    # ! queue max-size-buffers=1 leaky=downstream ! videorate ! video/x-raw, framerate=5/1 \
    PIPELINE = "appsrc name=app_src is-live=True format=3 \
        ! video/x-raw,width={width},height={height},format=BGR,framerate={fps}/1 \
        ! videoconvert \
        ! queue max-size-buffers=1 leaky=downstream \
        ! videorate \
        ! video/x-raw, framerate=10/1 \
        ! videoconvert  \
        ! video/x-raw,format=I420 \
        ! videoconvert \
        ! x264enc name=codec bitrate={bitrate} speed-preset=ultrafast  threads=2 \
        ! rtph264pay mtu={MTU} config-interval=1 pt=96 \
        ! udpsink host={host} port={port} sync=false".format(
        width=800,
        height=600,
        fps=10,
        bitrate=1000,
        MTU=1500,
        host="10.0.0.2",
        port=5000
    )

#     PIPELINE=""" videotestsrc is-live=true \
# ! video/x-raw,width=800,height=600,framerate=10/1 \
# ! videoconvert \
# ! queue max-size-buffers=1 leaky=downstream \
# ! videorate \
# ! video/x-raw, framerate=10/1 \
# ! videoconvert \
# ! video/x-raw,format=I420 \
# ! videoconvert \
# ! x264enc name=codec bitrate=500 speed-preset=ultrafast  \
# ! rtph264pay config-interval=1 pt=96 \
# ! udpsink host=10.0.0.2 port=5000 sync=true"""

    """
    gst-launch-1.0 -v udpsrc port=5000 ! application/x-rtp, encoding-name=H264, payload=96 ! rtph264depay  ! avdec_h264 ! videoconvert ! fpsdisplaysink
    """

    global pipeline
    pipeline = Gst.parse_launch(PIPELINE)
    global app_source
    app_source = pipeline.get_by_name("app_src")

    bus = pipeline.get_bus()
    # allow bus to emit messages to main thread
    bus.add_signal_watch()
    # Start pipeline
    pipeline.set_state(Gst.State.PLAYING)
    # Init GObject loop to handle Gstreamer Bus Events
    loop = GLib.MainLoop()
    # Add handler to specific signal
    bus.connect("message", on_message, loop)
    try:
        loop.run()
    except Exception:
        traceback.print_exc()
        loop.quit()


def ndarray_to_gst_buffer(array: np.ndarray) -> Gst.Buffer:
    """Converts numpy array to Gst.Buffer"""
    buffer = Gst.Buffer.new_wrapped(array.tobytes())
    buffer.pts = Gst.CLOCK_TIME_NONE
    # buffer.dts = Gst.CLOCK_TIME_NONE
    # buffer.pts = Gst.until_uint64_scale(pts, Gst.SECOND, 1)
    return buffer


# endregion

PARAM_BITRATE = "bitrate"
PARAM_MAX_QP = "max_qp"
NAME = "stream_node"


class MyNode(Node):
    def __init__(self):
        super().__init__(NAME)
        self.g = ReentrantCallbackGroup()
        self._init_parameters()
        self._init_subscribers()
        self.execute_gst()
        self.callback_group = ReentrantCallbackGroup()
        self.cv_br = CvBridge()
        self.add_on_set_parameters_callback(self.parameters_handler)

    def parameters_handler(self, params: List[Parameter]):
        success = True
        for param in params:
            try:
                if param.name == PARAM_BITRATE:
                    change_bitrate(param.value)
                if param.name == PARAM_MAX_QP:
                    change_codec_prop("qp-max", param.value)

            except:
                self.get_logger().error("Failed to update parameter")
                success = False
        return SetParametersResult(successful=success)

    def execute_gst(self):
        bitrate = self.get_parameter(PARAM_BITRATE).value
        gst_thread = Thread(
            target=gst_runner, args=(bitrate,), daemon=True, name="gst_t"
        )
        gst_thread.start()
        self.get_logger().info("gst run")

    def _init_clients(self): ...

    def _init_publishers(self): ...

    def _init_timers(self): ...

    def _init_parameters(self):
        bitrate_descriptor = ParameterDescriptor(
            description="stream bitrate",
            type=ParameterType.PARAMETER_INTEGER,
            integer_range=[IntegerRange(from_value=100, to_value=700)],
        )
        self.declare_parameter(PARAM_BITRATE, value=300, descriptor=bitrate_descriptor)

        self.declare_parameter(PARAM_MAX_QP, value=50)

    def _init_subscribers(self):

        self.img_sub = self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.image_handler,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.g,
        )

    def image_handler(self, msg: Image):
        frame = self.cv_br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imshow("sss", frame)
        if app_source:
            app_source.emit("push-buffer", ndarray_to_gst_buffer(frame))


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    executer = MultiThreadedExecutor()
    executer.add_node(node)
    executer.spin()

    node.destroy_node()
    executer.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

# edgeai-robotics-sdk