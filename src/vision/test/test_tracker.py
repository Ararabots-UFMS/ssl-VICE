"""
Regression tests for the vision filters and the object tracker.

The filter tests target the ExtendedKalmanFilter classes rather than the plain
KalmanFilter ones: ObjectTracker instantiates the EKFs, so those are the filters that
actually run on the field.
"""

import sys
import time
from unittest.mock import MagicMock

import numpy as np
import pytest

# The tracker imports rosidl-generated messages and the compiled ssl-vision protobufs
# purely for type annotations; neither is needed to exercise the filtering logic, and
# requiring them would mean a full ROS2 + protobuf build just to run these tests.
for _name in ("system_interfaces", "movement_interfaces"):
    if _name not in sys.modules:
        _mock = MagicMock(name=_name)
        sys.modules[_name] = _mock
        sys.modules[_name + ".msg"] = _mock.msg

if "vision.proto.messages_robocup_ssl_wrapper_pb2" not in sys.modules:
    sys.modules["vision.proto.messages_robocup_ssl_wrapper_pb2"] = MagicMock(
        name="messages_robocup_ssl_wrapper_pb2"
    )

from vision.kalman_filter import (  # noqa: E402
    ExtendedKalmanFilterClass1D,
    ExtendedKalmanFilterClass2D,
)
from vision.tracker import ID, ObjectTracker  # noqa: E402

FRICTION = 0.01


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
    it has never measured, and 1 mm/s confidence in a velocity it has never seen. With a
    realistic measurement noise the resulting gain is tiny, so the estimate crawls out of
    the origin and publishes metres of error the whole way.
    """

    def test_2d_filter_starts_at_the_first_detection(self):
        kf = ExtendedKalmanFilterClass2D(friction=FRICTION)
        kf.initialize(3000.0, 2000.0)

        assert kf.x[0, 0] == pytest.approx(3000.0)
        assert kf.x[1, 0] == pytest.approx(2000.0)
        # One frame carries no velocity information.
        assert kf.x[2, 0] == pytest.approx(0.0)
        assert kf.x[3, 0] == pytest.approx(0.0)

    def test_2d_filter_widens_the_velocity_covariance(self):
        """Seeding the state without widening P is what makes the filter ignore motion."""
        kf = ExtendedKalmanFilterClass2D(friction=FRICTION)
        kf.initialize(0.0, 0.0)

        assert kf.P[0, 0] == pytest.approx(kf.R[0, 0])
        assert kf.P[2, 2] > 1e4

    def test_2d_filter_acquires_immediately(self):
        target = (3000.0, 2000.0)
        kf = ExtendedKalmanFilterClass2D(friction=FRICTION)
        kf.initialize(*target)

        for _ in range(10):
            kf.update(np.matrix([[target[0]], [target[1]]]))
            kf.predict(1 / 60.0)

        error = float(np.hypot(kf.x[0, 0] - target[0], kf.x[1, 0] - target[1]))
        assert error < 1.0

    def test_2d_filter_tracks_a_moving_robot(self):
        """With the process noise set to 1 mm/s^2 the velocity estimate never catches up."""
        kf = ExtendedKalmanFilterClass2D(friction=FRICTION)
        kf.initialize(0.0, 0.0)
        dt, speed = 1 / 60.0, 600.0

        for k in range(1, 31):
            kf.predict(dt)
            kf.update(np.matrix([[k * speed * dt], [0.0]]))

        assert float(kf.x[2, 0]) == pytest.approx(speed, rel=0.15)

    def test_1d_filter_starts_at_the_first_orientation(self):
        kf = ExtendedKalmanFilterClass1D(friction=FRICTION)
        kf.initialize(2.5)

        assert kf.x[0, 0] == pytest.approx(2.5)
        assert kf.x[1, 0] == pytest.approx(0.0)

    def test_new_objects_are_seeded_from_their_first_detection(self):
        tracker = ObjectTracker(max_time_undetected=0.5)
        frame = FakeFrame(
            t_capture=1000.0,
            robots_blue=[FakeDetection(3, -1500.0, 900.0, orientation=1.2)],
        )

        tracker.update(FakePacket(frame), wall_stamp=1000.0)

        tracked = tracker.objects[ID(3, is_ball=False, is_blue=True)]
        assert tracked.KF.x[0, 0] == pytest.approx(-1500.0)
        assert tracked.KF.x[1, 0] == pytest.approx(900.0)
        assert tracked.orientation_KF.x[0, 0] == pytest.approx(1.2)


class TestStamps:
    """
    The published estimates describe the capture instant, not the moment they are
    published. Consumers difference wall_stamp against their own clock to age the data;
    without it the controller cannot tell a fresh pose from a 200 ms old one.
    """

    def test_stamps_track_the_last_processed_packet(self):
        tracker = ObjectTracker(max_time_undetected=0.5)
        frame = FakeFrame(t_capture=1234.5, robots_blue=[FakeDetection(0, 0.0, 0.0)])

        tracker.update(FakePacket(frame), wall_stamp=999.25)

        assert tracker.last_capture_stamp == pytest.approx(1234.5)
        assert tracker.last_wall_stamp == pytest.approx(999.25)

    def test_stamps_are_unset_before_the_first_packet(self):
        """None distinguishes "no data yet" from "captured at t = 0"."""
        tracker = ObjectTracker(max_time_undetected=0.5)

        assert tracker.last_capture_stamp is None

    def test_a_large_capture_stamp_does_not_blow_up_the_state(self):
        """A dt of a whole Unix epoch used to drive the state to ~1e17 mm."""
        tracker = ObjectTracker(max_time_undetected=0.5)
        frame = FakeFrame(t_capture=1.7e9, robots_blue=[FakeDetection(0, 500.0, 0.0)])

        tracker.update(FakePacket(frame), wall_stamp=1.7e9)

        tracked = tracker.objects[ID(0, is_ball=False, is_blue=True)]
        assert float(tracked.KF.x[0, 0]) == pytest.approx(500.0, abs=1.0)


class TestObjectDeletion:
    def _packet(self, ids, t=1000.0):
        return FakePacket(
            FakeFrame(t, robots_blue=[FakeDetection(i, 0.0, 0.0) for i in ids])
        )

    def test_objects_stay_while_they_are_still_seen(self):
        tracker = ObjectTracker(max_time_undetected=0.5)
        tracker.update(self._packet(range(4)), wall_stamp=1000.0)
        tracker.update(self._packet(range(4)), wall_stamp=1000.1)

        assert len(tracker.objects) == 4

    def test_an_occluded_object_is_dropped_once_it_times_out(self):
        tracker = ObjectTracker(max_time_undetected=0.05)
        tracker.update(self._packet([0, 1]), wall_stamp=1000.0)
        assert len(tracker.objects) == 2

        time.sleep(0.06)
        tracker.update(self._packet([0]), wall_stamp=1000.1)

        assert list(tracker.objects) == [ID(0, is_ball=False, is_blue=True)]

    def test_a_briefly_occluded_object_survives_but_loses_confidence(self):
        tracker = ObjectTracker(max_time_undetected=5.0)
        tracker.update(self._packet([0, 1]), wall_stamp=1000.0)
        tracker.update(self._packet([0]), wall_stamp=1000.1)

        missing = tracker.objects[ID(1, is_ball=False, is_blue=True)]
        assert len(tracker.objects) == 2
        assert missing.confidence == 0


class TestBallSelection:
    def test_the_most_confident_ball_is_tracked(self):
        tracker = ObjectTracker(max_time_undetected=0.5)
        balls = [
            FakeDetection(0, 0.0, 0.0, confidence=0.2),
            FakeDetection(0, 2500.0, -1000.0, confidence=0.9),
        ]
        tracker.update(FakePacket(FakeFrame(1000.0, balls=balls)), wall_stamp=1000.0)

        ball = tracker.objects[ID(0, is_ball=True)]
        assert ball.KF.x[0, 0] == pytest.approx(2500.0)


class TestOrientationTuning:
    """
    The orientation filter feeds the inverse kinematics, so heading error rotates the
    executed velocity sideways. The old tuning trusted a constant-rate model far more
    than the measurements, which is only correct while the rate is actually constant.
    """

    DT = 1 / 60.0
    MEASUREMENT_SD = 0.01  # rad, ssl-vision orientation noise
    ALPHA = 50.0           # rad/s², angular acceleration
    RATE = 10.0            # rad/s

    def _spin(self):
        """Accelerate to RATE, hold, decelerate back to rest."""
        ramp = int(self.RATE / self.ALPHA / self.DT)
        steps = 180
        angle, omega, out = 0.0, 0.0, []
        for k in range(steps):
            if k < ramp:
                alpha = self.ALPHA
            elif k < steps - ramp:
                alpha = 0.0
            else:
                alpha = -self.ALPHA
            omega += alpha * self.DT
            angle += omega * self.DT
            out.append(angle)
        return out

    def _worst_error(self, kf, seed=3):
        wrap = ExtendedKalmanFilterClass1D._wrap_angle
        rng = np.random.default_rng(seed)
        angles = self._spin()
        kf.initialize(wrap(angles[0]))

        worst = 0.0
        for angle in angles:
            noisy = wrap(angle + rng.normal(0.0, self.MEASUREMENT_SD))
            kf.predict(self.DT)
            kf.update(np.matrix([[noisy]]))
            worst = max(worst, abs(wrap(kf.x[0, 0] - wrap(angle))))
        return np.degrees(worst)

    def test_the_default_tuning_holds_heading_through_a_spin(self):
        worst = self._worst_error(ExtendedKalmanFilterClass1D(friction=FRICTION))

        # 5deg at 2000mm/s is ~175mm/s of sideways velocity through the kinematics.
        assert worst < 5.0

    def test_a_still_robot_is_still_smoothed(self):
        rng = np.random.default_rng(5)
        kf = ExtendedKalmanFilterClass1D(friction=FRICTION)
        kf.initialize(0.0)

        errors = []
        for _ in range(600):
            kf.predict(self.DT)
            kf.update(np.matrix([[rng.normal(0.0, self.MEASUREMENT_SD)]]))
            errors.append(abs(kf.x[0, 0]))

        # Responsiveness costs some jitter, but the estimate must still beat the raw
        # measurement it is filtering.
        assert np.mean(errors) < self.MEASUREMENT_SD
