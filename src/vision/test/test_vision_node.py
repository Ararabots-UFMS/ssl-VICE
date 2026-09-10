"""
Tests for the vision node's socket draining.

Reading one packet per tick tied the drain rate to the cost of filtering. Once that
cost overtook the camera rate the kernel buffer filled, and estimates arrived about a
second old while still being stamped as fresh. The node now empties the socket every
tick and folds in at most one frame per processing period, keeping only the newest
packet from each camera.
"""

import sys
from unittest.mock import MagicMock

import pytest

for _name in ("system_interfaces", "movement_interfaces"):
    if _name not in sys.modules:
        _mock = MagicMock(name=_name)
        sys.modules[_name] = _mock
        sys.modules[_name + ".msg"] = _mock.msg

for _name in (
    "vision.proto.messages_robocup_ssl_wrapper_pb2",
    "vision.proto.messages_robocup_ssl_geometry_pb2",
):
    if _name not in sys.modules:
        sys.modules[_name] = MagicMock(name=_name)

from vision.vision_node import Vision  # noqa: E402


class FakePacket:
    def __init__(self, camera_id=None, geometry=False, tag=None):
        self.camera_id = camera_id
        self._geometry = geometry
        self.geometry = MagicMock() if geometry else None
        self.tag = tag
        self.detection = MagicMock()
        self.detection.camera_id = camera_id

    def HasField(self, name):
        if name == "geometry":
            return self._geometry
        return not self._geometry


def _node(packets, process_hz=60.0, now=1000.0):
    node = Vision.__new__(Vision)
    node.verbose = False
    node.frequency_tracker_process = process_hz
    node._pending_frame = {}
    node._last_process = 0.0
    node._stale_packets = 0
    node._last_stale_report = 0.0
    node.tracker = MagicMock()
    node.publish_geometry = MagicMock()
    node.get_logger = MagicMock()

    node._clock_now = now
    node.get_clock = lambda: MagicMock(
        now=lambda: MagicMock(nanoseconds=int(node._clock_now * 1e9))
    )
    queue = list(packets)
    node.client = MagicMock()
    node.client.receive = lambda: queue.pop(0) if queue else None
    node._queue = queue
    return node


class TestDraining:
    def test_the_whole_socket_is_emptied_in_one_tick(self):
        node = _node([FakePacket(camera_id=i % 4) for i in range(40)])

        node._drain_socket()

        assert node._queue == []

    def test_only_the_newest_packet_from_each_camera_is_kept(self):
        """Working through a backlog is how the estimates fall behind; drop it instead."""
        node = _node([
            FakePacket(camera_id=0, tag="old"),
            FakePacket(camera_id=1, tag="old"),
            FakePacket(camera_id=0, tag="new"),
            FakePacket(camera_id=1, tag="new"),
        ])

        node._drain_socket()

        assert sorted(node._pending_frame) == [0, 1]
        assert [p.tag for p, _ in node._pending_frame.values()] == ["new", "new"]

    def test_superseded_packets_are_counted(self):
        node = _node([FakePacket(camera_id=0) for _ in range(5)])

        node._drain_socket()

        assert node._stale_packets == 4

    def test_geometry_is_published_and_not_tracked(self):
        node = _node([FakePacket(geometry=True), FakePacket(camera_id=2)])

        node._drain_socket()

        node.publish_geometry.assert_called_once()
        assert sorted(node._pending_frame) == [2]

    def test_a_flood_cannot_hold_the_executor(self):
        node = _node([FakePacket(camera_id=0) for _ in range(Vision.MAX_PACKETS_PER_DRAIN * 2)])

        node._drain_socket()

        assert len(node._queue) == Vision.MAX_PACKETS_PER_DRAIN


class TestProcessing:
    def test_one_frame_is_folded_in_per_period(self):
        node = _node([FakePacket(camera_id=i) for i in range(4)])
        node._drain_socket()

        node._process_pending_frame()

        assert node.tracker.update_frame.call_count == 1
        packets = node.tracker.update_frame.call_args[0][0]
        assert len(packets) == 4
        assert node._pending_frame == {}

    def test_filtering_does_not_run_faster_than_the_period(self):
        node = _node([FakePacket(camera_id=0)], process_hz=60.0)
        node._drain_socket()
        node._process_pending_frame()

        node._queue.append(FakePacket(camera_id=1))
        node._clock_now += 0.001          # well inside the 1/60 s period
        node._drain_socket()
        node._process_pending_frame()

        assert node.tracker.update_frame.call_count == 1
        # the packet is held, not dropped
        assert sorted(node._pending_frame) == [1]

    def test_the_next_period_folds_in_what_was_held(self):
        node = _node([FakePacket(camera_id=0)], process_hz=60.0)
        node._drain_socket()
        node._process_pending_frame()

        node._queue.append(FakePacket(camera_id=1))
        node._clock_now += 0.05
        node._drain_socket()
        node._process_pending_frame()

        assert node.tracker.update_frame.call_count == 2

    def test_nothing_pending_means_no_call(self):
        node = _node([])

        node._drain_socket()
        node._process_pending_frame()

        node.tracker.update_frame.assert_not_called()

    def test_the_frame_is_stamped_when_its_packets_arrived(self):
        node = _node([FakePacket(camera_id=0)], now=1234.5)

        node._drain_socket()
        node._process_pending_frame()

        assert node.tracker.update_frame.call_args[1]["wall_stamp"] == pytest.approx(1234.5)


class TestStaleReporting:
    def test_it_warns_when_the_cameras_outrun_the_tracker(self):
        node = _node([], process_hz=60.0)
        node._last_stale_report = 1000.0
        node._stale_packets = 2000          # far above 60/s over the period
        node._clock_now = 1000.0 + Vision.STALE_REPORT_PERIOD + 0.1

        node._report_stale_packets(node._clock_now)

        node.get_logger().warning.assert_called_once()
        assert node._stale_packets == 0

    def test_an_occasional_superseded_packet_is_not_worth_a_warning(self):
        node = _node([], process_hz=60.0)
        node._last_stale_report = 1000.0
        node._stale_packets = 3
        node._clock_now = 1000.0 + Vision.STALE_REPORT_PERIOD + 0.1

        node._report_stale_packets(node._clock_now)

        node.get_logger().warning.assert_not_called()
