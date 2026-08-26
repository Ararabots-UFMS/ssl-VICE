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


class TestOrientationTuning:
    """
    The orientation filter feeds the inverse kinematics, so heading error rotates the
    executed velocity sideways. Its old tuning trusted a constant-rate model far more
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
        rng = np.random.default_rng(seed)
        angles = self._spin()
        kf.initialize(KalmanFilterClass1D._wrap_angle(angles[0]))

        worst = 0.0
        for angle in angles:
            noisy = KalmanFilterClass1D._wrap_angle(
                angle + rng.normal(0.0, self.MEASUREMENT_SD)
            )
            kf.predict(self.DT)
            kf.update(np.matrix([[noisy]]))
            error = KalmanFilterClass1D._wrap_angle(
                kf.x[0, 0] - KalmanFilterClass1D._wrap_angle(angle)
            )
            worst = max(worst, abs(error))
        return np.degrees(worst)

    def test_the_default_tuning_holds_heading_through_a_spin(self):
        worst = self._worst_error(KalmanFilterClass1D())

        # 5deg at 2000mm/s is ~175mm/s of sideways velocity through the kinematics.
        assert worst < 5.0

    def test_the_old_tuning_would_not(self):
        worst = self._worst_error(KalmanFilterClass1D(a_sd=0.1, sd_acceleration=1.0))

        assert worst > 20.0

    def test_a_still_robot_is_still_smoothed(self):
        rng = np.random.default_rng(5)
        kf = KalmanFilterClass1D()
        kf.initialize(0.0)

        errors = []
        for _ in range(600):
            kf.predict(self.DT)
            kf.update(np.matrix([[rng.normal(0.0, self.MEASUREMENT_SD)]]))
            errors.append(abs(kf.x[0, 0]))

        # Responsiveness costs some jitter, but the estimate must still beat the raw
        # measurement it is filtering.
        assert np.mean(errors) < self.MEASUREMENT_SD
