from vision.vision_client import Client
from vision.tracker import ObjectTracker
from vision.world_message import wrap_geo_message, wrap_message

from typing import Optional

import rclpy
from rclpy.node import Node

from google.protobuf import text_format
from vision.proto.messages_robocup_ssl_geometry_pb2 import SSL_GeometryData

from system_interfaces.msg import VisionMessage, VisionGeometry


class Vision(Node):
    """VICE Vision Node, connects and receives data from ssl-vision"""

    # Safety valve on one drain pass, so a flood cannot hold the executor. The next
    # tick continues where this one stopped.
    MAX_PACKETS_PER_DRAIN = 200
    # Seconds between "cameras outrunning the tracker" warnings.
    STALE_REPORT_PERIOD = 5.0

    def __init__(self):
        super().__init__("visionNode")

        # Declaration of parameters with default values.
        default_params = {
            "ip": "224.5.23.2",
            "port": 10006,
            "verbose": False,
            "interface_ip": "",
            "socket_timeout": 0.0,
            "num_cams": 4,
            "max_time_undetected": 0.5,
            "frequency_timer_publish": 60.0,
            # How often the socket is drained. Draining is cheap and must outpace the
            # cameras, or packets pile up in the kernel buffer and every estimate ages.
            "frequency_tracker_update": 1000.0,
            # How often a drained frame is folded into the filters. Matched to the
            # camera frame rate: filtering faster than the cameras deliver only repeats
            # the prediction step over the same data.
            "frequency_tracker_process": 60.0,
            "friction": 0.01,
        }

        for name, default in default_params.items():
            self.declare_parameter(name, default)

        # Retrieve already typed parameters.
        self.ip = self.get_parameter("ip").value
        self.port = self.get_parameter("port").value
        self.verbose = self.get_parameter("verbose").value
        self.interface_ip = self.get_parameter("interface_ip").value
        self.socket_timeout = self.get_parameter("socket_timeout").value
        self.num_cams = self.get_parameter("num_cams").value
        self.max_time_undetected = self.get_parameter("max_time_undetected").value
        self.frequency_timer_publish = self.get_parameter("frequency_timer_publish").value
        self.frequency_tracker_update = self.get_parameter("frequency_tracker_update").value
        self.frequency_tracker_process = self.get_parameter("frequency_tracker_process").value
        self.friction = self.get_parameter("friction").value

        # Newest packet seen from each camera since the last time a frame was folded in,
        # as {camera_id: (packet, wall_stamp)}. Older packets from the same camera are
        # dropped rather than queued: a stale view of the field is worth nothing, and
        # working through a backlog is how the estimates fall a second behind.
        self._pending_frame = {}
        self._last_process = 0.0
        self._stale_packets = 0
        self._last_stale_report = 0.0
        
        self.client = Client(
            ip=self.ip,
            port=self.port,
            interface_ip=self.interface_ip if self.interface_ip else None,
            timeout=self.socket_timeout,
            logger=self.get_logger(),
        )
        self.get_logger().info(f"Binding client on {self.ip}:{self.port}")
        self.client.connect()

        # Setting ROS publisher.
        # TODO: Find optimal queue size...
        self.publisher = self.create_publisher(VisionMessage, "visionTopic", 10)
        self.geometry_publisher = self.create_publisher(
            VisionGeometry, "geometryTopic", 10
        )

        self.tracker = ObjectTracker(max_time_undetected=self.max_time_undetected)

        # TODO: Find the optimal timer.
        # Timer slow to publisher messages ROS.
        self.publish_timer = self.create_timer(1.0/self.frequency_timer_publish, self.publish_vision)
        # Timer fast to process vision packets.
        self.tracker_timer = self.create_timer(1.0/self.frequency_tracker_update, self.update_tracker)

    def update_tracker(self):
        """
        Empty the socket, then fold at most one frame per processing period into the
        filters.

        Reading a single packet per tick tied the drain rate to the cost of filtering.
        On a full field that cost overtook the camera rate, the kernel buffer filled,
        and every estimate arrived about a second old while still being stamped as
        fresh — which nothing downstream could detect.
        """
        try:
            self._drain_socket()
            self._process_pending_frame()

        except KeyboardInterrupt:
            self.get_logger().info("KeyboardInterrupt received, shutting down...")
            raise

        except Exception as exception:
            self.get_logger().warning(f"Error receiving data: {exception}")

    def _drain_socket(self):
        """Take every packet the socket is holding, keeping the newest per camera."""
        for _ in range(self.MAX_PACKETS_PER_DRAIN):
            data = self.client.receive()
            if data is None:
                return

            if self.verbose:
                self.get_logger().info(text_format.MessageToString(data))

            if data.HasField("geometry"):
                self.publish_geometry(data.geometry)
                continue

            if not data.HasField("detection"):
                continue

            # Stamped on arrival, not when the frame is folded in, so the estimate
            # carries the instant it describes.
            now_sec = self.get_clock().now().nanoseconds / 1e9
            camera_id = data.detection.camera_id
            if camera_id in self._pending_frame:
                self._stale_packets += 1
            self._pending_frame[camera_id] = (data, now_sec)

    def _process_pending_frame(self):
        if not self._pending_frame:
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self._last_process < 1.0 / self.frequency_tracker_process:
            return
        self._last_process = now_sec

        frame = self._pending_frame
        self._pending_frame = {}
        self.tracker.update_frame(
            [packet for packet, _ in frame.values()],
            wall_stamp=max(stamp for _, stamp in frame.values()),
        )

        self._report_stale_packets(now_sec)

    def _report_stale_packets(self, now_sec):
        """
        Superseded packets are normal in ones and twos; a steady stream of them means
        the cameras are outrunning the filters and the field is only partly observed.
        """
        if now_sec - self._last_stale_report < self.STALE_REPORT_PERIOD:
            return

        elapsed = now_sec - self._last_stale_report
        if self._last_stale_report and self._stale_packets:
            rate = self._stale_packets / elapsed
            if rate > self.frequency_tracker_process:
                self.get_logger().warning(
                    f"Discarding {rate:.0f} superseded vision packets/s: the cameras "
                    f"are outrunning the tracker."
                )

        self._last_stale_report = now_sec
        self._stale_packets = 0


    def set_filter_param(
        self,
        x_sd: Optional[float] = None,
        y_sd: Optional[float] = None,
        a_sd: Optional[float] = None,
        u_x: Optional[float] = None,
        u_y: Optional[float] = None,
        u_a: Optional[float] = None,
        acceleration_sd_2d: Optional[float] = None,
        acceleration_sd_1d: Optional[float] = None,
        friction: Optional[float] = None,
    ):
        
        for object_ in self.tracker.objects.values():
            object_.KF.set_param(x_sd, y_sd, u_x, u_y, acceleration_sd_2d, friction)
            if not object_.id.is_ball:
                object_.orientation_KF.set_param(a_sd, u_a, acceleration_sd_1d, friction)

    def publish_vision(self):
        message = wrap_message(
            self.tracker.objects,
            self.get_logger(),
            capture_stamp=self.tracker.last_capture_stamp or 0.0,
            wall_stamp=self.tracker.last_wall_stamp,
        )

        # Validate message before publishing (catch garbage/overflow)
        if not self._is_valid_vision_message(message):
            self.get_logger().warning("Skipping invalid vision message")
            return

        if rclpy.ok():
            self.publisher.publish(message)

    def _is_valid_vision_message(self, message: VisionMessage) -> bool:
        """Check if vision message contains reasonable data"""
        max_reasonable_pos = 10000.0  # 10m in mm
        
        for robot in list(message.yellow_robots) + list(message.blue_robots):
            if (
                abs(robot.position_x) > max_reasonable_pos
                or abs(robot.position_y) > max_reasonable_pos
            ):
                self.get_logger().warning(
                    f"Invalid robot position detected: ({robot.position_x}, {robot.position_y})"
                )
                return False
        
        for ball in message.balls:
            if abs(ball.position_x) > max_reasonable_pos or abs(ball.position_y) > max_reasonable_pos:
                self.get_logger().warning(f"Invalid ball position: ({ball.position_x}, {ball.position_y})")
                return False
        
        return True

    def publish_geometry(self, message: SSL_GeometryData):
        message: VisionGeometry = wrap_geo_message(message)

        if rclpy.ok():
            self.geometry_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = Vision()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
