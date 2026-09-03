import sys
from unittest.mock import MagicMock

# Vector2D and the controller import rosidl-generated message types for their
# to_msg/from_msg helpers, which these tests do not exercise; stubbing them keeps the
# suite runnable without a full ROS2 build.
for _name in ("movement_interfaces", "system_interfaces"):
    if _name not in sys.modules:
        _mock = MagicMock(name=_name)
        sys.modules[_name] = _mock
        sys.modules[_name + ".msg"] = _mock.msg

import pytest

from new_movement.entities.motion import MotionState

from utils.math_util import Vector2D

from control.pid_controller import (
    DEFAULT_SLEW_LIMIT,
    PIDController,
    RobotTrajectoryController,
    Vector2DTrajectoryController,
)


DT = 0.02


def _follow(controller, target_pos, target_vel, current_pos, current_vel, ticks=1):
    for _ in range(ticks):
        output = controller.compute_trajectory_following(
            target_pos, target_vel, current_pos, current_vel, DT
        )
    return output


class TestFeedback:
    """
    kp=1 with everything in metres meant a 1m error bought a 1m/s correction, and kd=0
    meant no velocity feedback at all: feedforward replay with a weak trim.
    """

    def test_a_robot_on_its_reference_gets_the_feedforward(self):
        controller = PIDController(kp=4.0, ki=0.0, kd=0.4)

        output = _follow(controller, 1.0, 1.5, 1.0, 1.5)

        assert output == pytest.approx(1.5)

    def test_position_error_is_corrected_proportionally(self):
        controller = PIDController(kp=4.0, ki=0.0, kd=0.0)

        # 100mm behind, no velocity error, generous slew so it is not the binding limit.
        controller.slew_limit = 1e6
        output = _follow(controller, 1.0, 0.0, 0.9, 0.0)

        assert output == pytest.approx(0.4)

    def test_velocity_error_is_corrected(self):
        controller = PIDController(kp=0.0, ki=0.0, kd=0.4)
        controller.slew_limit = 1e6

        # In the right place, but 1 m/s too slow.
        output = _follow(controller, 1.0, 1.0, 1.0, 0.0)

        assert output == pytest.approx(1.0 + 0.4)


class TestSlewLimit:
    """
    Without this, a reference discontinuity reaches the wheels as a step the robot can
    only answer with slip.
    """

    def test_the_command_cannot_step_further_than_the_limit(self):
        controller = PIDController(kp=4.0, ki=0.0, kd=0.0)

        # Robot at rest, reference a long way off: unclamped this saturates at 3 m/s.
        first = _follow(controller, 5.0, 0.0, 0.0, 0.0)
        second = _follow(controller, 5.0, 0.0, 0.0, 0.0)

        assert first == pytest.approx(DEFAULT_SLEW_LIMIT * DT)
        assert second - first == pytest.approx(DEFAULT_SLEW_LIMIT * DT)

    def test_it_ramps_rather_than_blocks(self):
        controller = PIDController(kp=4.0, ki=0.0, kd=0.0)

        output = _follow(controller, 5.0, 0.0, 0.0, 0.0, ticks=100)

        assert output == pytest.approx(controller.output_limit)

    def test_a_fresh_controller_resumes_from_the_measured_velocity(self):
        controller = PIDController(kp=4.0, ki=0.0, kd=0.0)

        # Already moving at 2 m/s: the command must not be dragged back towards zero.
        output = _follow(controller, 5.0, 2.0, 0.0, 2.0)

        assert output == pytest.approx(2.0 + DEFAULT_SLEW_LIMIT * DT)

    def test_reset_reseeds_instead_of_stepping(self):
        controller = PIDController(kp=4.0, ki=0.0, kd=0.0)
        _follow(controller, 5.0, 0.0, 0.0, 0.0, ticks=50)

        controller.reset()
        output = _follow(controller, 5.0, 0.0, 0.0, 1.2)

        assert output == pytest.approx(1.2 + DEFAULT_SLEW_LIMIT * DT)


class TestRobotTrajectoryController:
    def test_defaults_carry_through_to_each_axis(self):
        robot_controller = RobotTrajectoryController()
        controller = robot_controller.get_controller(3)

        assert controller.x_controller.kp == robot_controller.default_kp
        assert controller.y_controller.kd == robot_controller.default_kd
        assert controller.x_controller.slew_limit == robot_controller.default_slew_limit

    def test_a_target_jump_resets_without_a_command_step(self):
        robot_controller = RobotTrajectoryController()
        current = MotionState(Vector2D(0.0, 0.0), Vector2D(1.0, 0.0))

        near = MotionState(Vector2D(1.0, 0.0), Vector2D(1.0, 0.0))
        for _ in range(20):
            robot_controller.compute_trajectory_command(1, near, current, DT)

        # Well past reset_threshold, so the controller resets.
        far = MotionState(Vector2D(-4.0, 0.0), Vector2D(0.0, 0.0))
        command = robot_controller.compute_trajectory_command(1, far, current, DT)

        assert abs(command.x - current.velocity.x) <= DEFAULT_SLEW_LIMIT * DT + 1e-9

    def test_axes_are_controlled_independently(self):
        controller = Vector2DTrajectoryController()
        target = MotionState(Vector2D(0.0, 1.0), Vector2D(0.0, 0.5))
        current = MotionState(Vector2D(0.0, 0.0), Vector2D(0.0, 0.0))

        command = controller.compute_trajectory_following(target, current, DT)

        assert command.x == pytest.approx(0.0)
        assert command.y > 0.0
