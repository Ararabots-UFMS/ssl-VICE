import numpy as np
import pytest

from vision.kalman_filter import KalmanFilterClass1D, KalmanFilterClass2D
from vision.tracker import ID, ObjectTracker


class FakeDetection:
    """Stands in for one robot inside an ssl-vision detection frame."""

    def __init__(self, robot_id, x, y, orientation=0.0, confidence=1.0):
        self.robot_id = robot_id
        self.x = x
        self.y = y
        self.orientation = orientation
        self.confidence = confidence


class FakeFrame:
    def __init__(self, t_capture, robots_blue=(), robots_yellow=(), balls=()):
        self.t_capture = t_capture
        self.robots_blue = list(robots_blue)
        self.robots_yellow = list(robots_yellow)
        self.balls = list(balls)


class FakePacket:
    def __init__(self, detection):
        self.detection = detection


class TestFilterInitialization:
    """
    A filter that starts at the origin with P = I claims 1 mm confidence in a position
    it has never measured. With a realistic measurement noise the resulting gain is
    tiny, so the estimate crawls out of the origin over about a second and publishes
    metres of error the whole way.
    """

    def test_2d_filter_starts_at_the_first_detection(self):
        kf = KalmanFilterClass2D()
        kf.initialize(3000.0, 2000.0)

        assert kf.x[0, 0] == pytest.approx(3000.0)
        assert kf.x[1, 0] == pytest.approx(2000.0)
        # One frame carries no velocity information.
        assert kf.x[2, 0] == pytest.approx(0.0)
        assert kf.x[3, 0] == pytest.approx(0.0)

    def test_2d_filter_acquires_immediately(self):
        target = (3000.0, 2000.0)
        kf = KalmanFilterClass2D()
        kf.initialize(*target)

        for _ in range(10):
            kf.update(np.matrix([[target[0]], [target[1]]]))
            kf.predict(1 / 60.0)

        error = float(np.hypot(kf.x[0, 0] - target[0], kf.x[1, 0] - target[1]))
        assert error < 1.0

    def test_1d_filter_starts_at_the_first_orientation(self):
        kf = KalmanFilterClass1D()
        kf.initialize(2.5)

        assert kf.x[0, 0] == pytest.approx(2.5)
        assert kf.x[1, 0] == pytest.approx(0.0)

    def test_add_object_seeds_the_filters(self):
        tracker = ObjectTracker(max_frame_skipped=30)
        tracker.add_object(
            ID(3, is_ball=False, is_blue=True), -1500.0, 900.0, 1.0, orientation=1.2
        )

        tracked = tracker.objects[0]
        assert tracked.KF.x[0, 0] == pytest.approx(-1500.0)
        assert tracked.KF.x[1, 0] == pytest.approx(900.0)
        assert tracked.orientation_KF.x[0, 0] == pytest.approx(1.2)


class TestUpdateOrdering:
    """
    Every filter must be propagated to the current capture instant before the new
    detections are fused in. Fusing first and propagating afterwards compares each
    measurement against a prior stretched by the previous interval, and leaves the
    published state extrapolated one frame past its own timestamp.
    """

    def _run(self, tracker, positions, dt=1 / 60.0, t0=1000.0):
        for k, x in enumerate(positions):
            frame = FakeFrame(
                t_capture=t0 + k * dt,
                robots_blue=[FakeDetection(0, x, 0.0)],
            )
            tracker.update(FakePacket(frame), wall_stamp=t0 + k * dt)
        return tracker.objects[0]

    def test_state_is_not_extrapolated_past_the_measurement(self):
        tracker = ObjectTracker(max_frame_skipped=30)
        dt = 1 / 60.0
        speed = 2000.0  # mm/s

        positions = [k * speed * dt for k in range(60)]
        tracked = self._run(tracker, positions, dt=dt)

        # The published estimate must sit on the last measurement, not a frame beyond
        # it. One frame of overshoot at this speed is ~33 mm.
        assert float(tracked.KF.x[0, 0]) == pytest.approx(positions[-1], abs=5.0)

    def test_stamps_track_the_last_processed_packet(self):
        tracker = ObjectTracker(max_frame_skipped=30)
        frame = FakeFrame(t_capture=1234.5, robots_blue=[FakeDetection(0, 0.0, 0.0)])

        tracker.update(FakePacket(frame), wall_stamp=999.25)

        assert tracker.last_time_stamp == pytest.approx(1234.5)
        assert tracker.last_wall_stamp == pytest.approx(999.25)

    def test_first_packet_does_not_propagate(self):
        tracker = ObjectTracker(max_frame_skipped=30)
        frame = FakeFrame(t_capture=1.7e9, robots_blue=[FakeDetection(0, 500.0, 0.0)])

        tracker.update(FakePacket(frame), wall_stamp=1.7e9)

        # dt of a whole Unix epoch used to drive the state to ~1e17 mm.
        assert tracker.dt == pytest.approx(0.0)
        assert float(tracker.objects[0].KF.x[0, 0]) == pytest.approx(500.0, abs=1.0)


class TestObjectDeletion:
    def test_occluded_objects_are_dropped_without_desyncing(self):
        tracker = ObjectTracker(max_frame_skipped=2)
        for i in range(4):
            tracker.add_object(ID(i, is_ball=False, is_blue=True), 0.0, 0.0, 1.0, 0.0)

        for _ in range(6):
            tracker.delete_undetected_objects([])
            assert len(tracker.objects_id) == len(tracker.objects)

        assert tracker.objects_id == []
