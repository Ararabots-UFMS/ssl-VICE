"""
Controller node without the ROS machinery, as in movement's test_movement_planner:
Node.__init__ and the create_* calls are patched out, parameters come from a dict.
"""
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest

from movement.entities.motion import MotionState
from movement.entities.trajectory import Trajectory
from movement.local_planner import TrajectoryGenerator
from movement.movement_tracker import reference_state

from utils.math_util import Vector2D

from control.control import (
    MAX_MEASUREMENT_AGE,
    MAX_PREDICTION_AGE,
    MAX_REFERENCE_AGE,
    PREDICTOR_TIMEOUT,
    Controller,
)

NOW = 100.0
DELAY, TAU = 0.04, 0.08

DEFAULT_PARAMS = {
    "control_mode": "predictive",
    "actuation_delay": DELAY,
    "velocity_time_constant": TAU,
    "robot_max_acceleration": 3.0,
    "slew_limit": 3.0,
    "vision_extra_latency": 0.0,
    "coast_max_time": 0.3,
}


class FakeTime:
    def __init__(self, seconds: float):
        self.nanoseconds = int(round(seconds * 1e9))

    def __sub__(self, other):
        return SimpleNamespace(nanoseconds=self.nanoseconds - other.nanoseconds)


def _set_clock(node, now_sec):
    node.get_clock = MagicMock(return_value=MagicMock(now=MagicMock(return_value=FakeTime(now_sec))))


def _controller(now_sec=NOW, **params) -> Controller:
    values = dict(DEFAULT_PARAMS, **params)
    with patch("rclpy.node.Node.__init__", return_value=None), \
         patch.object(Controller, "create_subscription", return_value=MagicMock()), \
         patch.object(Controller, "create_publisher", return_value=MagicMock()), \
         patch.object(Controller, "create_service", return_value=MagicMock()), \
         patch.object(Controller, "create_client", return_value=MagicMock()), \
         patch.object(Controller, "create_timer", return_value=MagicMock()), \
         patch.object(Controller, "declare_parameter", return_value=MagicMock()), \
         patch.object(Controller, "get_parameter",
                      side_effect=lambda name: MagicMock(value=values[name])), \
         patch.object(Controller, "get_logger", return_value=MagicMock()), \
         patch.object(Controller, "get_clock",
                      return_value=MagicMock(now=MagicMock(return_value=FakeTime(now_sec - 0.02)))):
        node = Controller()
    node.get_logger = MagicMock()
    node.publisher = MagicMock()
    _set_clock(node, now_sec)
    return node


def _robot(rid=1, x=0.0, y=0.0, vx=0.0, vy=0.0, orientation=0.0):
    return SimpleNamespace(id=rid, position_x=x, position_y=y, velocity_x=vx, velocity_y=vy,
                           orientation=orientation)


def _game_state(robots, stamp, command="NORMAL_START"):
    return SimpleNamespace(ally_robots=robots, vision_wall_stamp=stamp,
                           referee=SimpleNamespace(command=command))


def _trajectory(distance=2000.0) -> Trajectory:
    generator = TrajectoryGenerator()
    start = MotionState(Vector2D(0, 0), Vector2D(0, 0))
    goal = MotionState(Vector2D(distance, 0), Vector2D(0, 0))
    return Trajectory(generator.generate(start, goal))


def _reference(rid=1, trajectory=None, timestamp=0.3, wall_stamp=NOW - 0.01, handoff_stamp=50.0):
    trajectory = trajectory or _trajectory()
    msg = trajectory.to_msg(rid)
    msg.handoff_stamp = handoff_stamp
    state = trajectory.get_state(timestamp)
    return SimpleNamespace(robot_id=rid, pos=state.position, vel=state.velocity,
                           timestamp=timestamp, wall_stamp=wall_stamp, trajectory=msg)


def _spy(node, command=Vector2D(0.5, -0.25)):
    node.robot_controller.compute_trajectory_command = MagicMock(return_value=command)
    return node.robot_controller.compute_trajectory_command


def _published(node):
    return node.publisher.publish.call_args[0][0].robots


class TestPredictiveState:
    def test_the_state_is_the_prediction_at_now_plus_the_delay(self):
        node = _controller()
        node.game_state_callback(_game_state([_robot(x=10.0)], stamp=NOW - 0.03))
        node.control_references[1] = _reference()
        predictor = node.predictors[1]
        predictor.state_at = MagicMock(return_value=(1000.0, 2000.0, 300.0, -400.0))
        spy = _spy(node)

        node.timer_callback()

        predictor.state_at.assert_called_once_with(pytest.approx(NOW + DELAY))
        _, _, current, _ = spy.call_args[0]
        assert (current.position.x, current.position.y) == pytest.approx((1.0, 2.0))
        assert (current.velocity.x, current.velocity.y) == pytest.approx((0.3, -0.4))

    def test_vision_is_corrected_at_its_stamp_minus_the_extra_latency(self):
        node = _controller(vision_extra_latency=0.01)

        node.game_state_callback(_game_state([_robot(x=10.0, y=20.0)], stamp=NOW - 0.03))

        assert node.predictors[1].last_capture == pytest.approx(NOW - 0.04)
        assert node.predictors[1].state_at(NOW - 0.04)[:2] == (10.0, 20.0)

    def test_a_stale_capture_is_only_predicted_so_far(self):
        node = _controller()
        node.game_state_callback(_game_state([_robot()], stamp=NOW - 0.6))
        node.control_references[1] = _reference()
        node.predictors[1].state_at = MagicMock(return_value=(0.0, 0.0, 0.0, 0.0))
        _spy(node)

        node.timer_callback()

        node.predictors[1].state_at.assert_called_once_with(
            pytest.approx(NOW - 0.6 + MAX_PREDICTION_AGE))

    def test_the_published_commands_are_recorded(self):
        node = _controller()
        node.game_state_callback(_game_state([_robot()], stamp=NOW - 0.03))
        node.control_references[1] = _reference()
        _spy(node, Vector2D(0.5, -0.25))

        node.timer_callback()

        out = _published(node)[0]
        assert (out.linear_velocity_x, out.linear_velocity_y) == (0.5, -0.25)
        commands = list(node.predictors[1].model._commands)
        assert len(commands) == 1
        assert commands[0][0] == pytest.approx(NOW + DELAY)
        assert commands[0][1].tolist() == [500.0, -250.0]

    def test_halt_zeros_are_recorded_too(self):
        node = _controller()
        node.game_state_callback(_game_state([_robot()], stamp=NOW - 0.03, command="HALT"))
        node.control_references[1] = _reference()
        _spy(node, Vector2D(0.5, -0.25))

        node.timer_callback()

        assert node.predictors[1].model._commands[0][1].tolist() == [0.0, 0.0]

    def test_a_predictor_is_dropped_after_a_second_without_a_capture(self):
        node = _controller()
        node.game_state_callback(_game_state([_robot(1), _robot(2)], stamp=NOW - 0.03))

        _set_clock(node, NOW + PREDICTOR_TIMEOUT + 0.1)
        node.game_state_callback(_game_state([_robot(1)], stamp=NOW + PREDICTOR_TIMEOUT))

        assert sorted(node.predictors) == [1]


class TestPredictiveReference:
    def test_position_at_offset_plus_age_plus_delay_velocity_tau_later(self):
        node = _controller()
        node.game_state_callback(_game_state([_robot()], stamp=NOW - 0.03))
        trajectory = _trajectory()
        node.control_references[1] = _reference(trajectory=trajectory, timestamp=0.3,
                                                wall_stamp=NOW - 0.012)
        spy = _spy(node)

        node.timer_callback()

        _, target, _, _ = spy.call_args[0]
        t_ref = 0.3 + 0.012 + DELAY
        at = reference_state(trajectory, t_ref, 0.3)
        ahead = reference_state(trajectory, t_ref + TAU, 0.3)
        assert target.position.x == pytest.approx(at.position.x / 1000.0)
        assert target.velocity.x == pytest.approx(ahead.velocity.x / 1000.0)
        assert ahead.velocity.x != pytest.approx(at.velocity.x)

    @pytest.mark.parametrize("wall_stamp, age", [
        (NOW - 1.0, MAX_REFERENCE_AGE),   # stale: capped
        (NOW + 0.05, 0.0),                # ahead of this clock: no negative age
        (0.0, 0.0),                       # tracker that does not stamp references
    ])
    def test_the_reference_age_is_bounded(self, wall_stamp, age):
        node = _controller()
        trajectory = _trajectory()
        ref = _reference(trajectory=trajectory, timestamp=0.3, wall_stamp=wall_stamp)

        target = node._predictive_reference(1, ref, NOW)

        expected = reference_state(trajectory, 0.3 + age + DELAY, 0.3)
        assert target.position.x == pytest.approx(expected.position.x / 1000.0)

    def test_a_plan_is_decoded_once(self):
        node = _controller()
        ref = _reference()

        with patch("control.control.Trajectory.from_msg", wraps=Trajectory.from_msg) as decode:
            node._predictive_reference(1, ref, NOW)
            node._predictive_reference(1, _reference(timestamp=0.35), NOW)

        assert decode.call_count == 1

    def test_a_new_plan_with_the_same_handoff_stamp_is_decoded_again(self):
        node = _controller()
        first = _reference(trajectory=_trajectory(2000.0), handoff_stamp=50.0)
        second = _reference(trajectory=_trajectory(500.0), handoff_stamp=50.0)

        node._predictive_reference(1, first, NOW)
        target = node._predictive_reference(1, second, NOW)

        expected = reference_state(_trajectory(500.0), 0.3 + 0.01 + DELAY, 0.3)
        assert target.position.x == pytest.approx(expected.position.x / 1000.0)

    def test_without_a_plan_it_falls_back_to_the_published_point(self):
        node = _controller()
        ref = _reference()
        ref.trajectory.segments = []

        target = node._predictive_reference(1, ref, NOW)

        assert target.position.x == pytest.approx(ref.pos.x / 1000.0)
        assert target.velocity.x == pytest.approx(ref.vel.x / 1000.0)


class TestLegacy:
    def test_it_carries_the_vision_pose_forward_and_uses_the_reference_as_is(self):
        node = _controller(control_mode="legacy")
        node.game_state_callback(_game_state([_robot(x=1000.0, vx=500.0)], stamp=NOW - 0.05))
        ref = _reference()
        node.control_references[1] = ref
        spy = _spy(node)

        node.timer_callback()

        _, target, current, _ = spy.call_args[0]
        assert current.position.x == pytest.approx((1000.0 + 500.0 * 0.05) / 1000.0)
        assert current.velocity.x == pytest.approx(0.5)
        assert target.position.x == pytest.approx(ref.pos.x / 1000.0)
        assert target.velocity.x == pytest.approx(ref.vel.x / 1000.0)
        assert node.predictors == {}

    def test_the_carry_forward_is_capped(self):
        node = _controller(control_mode="legacy")
        node.game_state_callback(_game_state([_robot(x=0.0, vx=1000.0)], stamp=NOW - 5.0))
        node.control_references[1] = _reference()
        spy = _spy(node)

        node.timer_callback()

        assert spy.call_args[0][2].position.x == pytest.approx(MAX_MEASUREMENT_AGE)

    def test_an_unknown_mode_is_refused(self):
        with pytest.raises(ValueError):
            _controller(control_mode="fast")


def test_slew_limit_param_reaches_the_trajectory_controller():
    node = _controller(slew_limit=2.5)

    assert node.robot_controller.get_controller(1).x_controller.slew_limit == 2.5
