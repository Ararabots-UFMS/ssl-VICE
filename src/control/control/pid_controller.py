import time
from typing import Optional

from new_movement.entities.States import State, Vector2D


class PIDController:
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.previous_error: Optional[float] = None
        self.integral: float = 0.0
        self.previous_time: Optional[float] = None

        self.integral_limit: float = 0.03  # Anti Windup
        self.output_limit: float = 0.1  # Max velocity

    def update_params(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def reset(self):
        self.previous_error = None
        self.integral = 0.0
        self.previous_time = None

    def compute_trajectory_following(
        self,
        target_position: float,
        target_velocity: float,
        current_position: float,
        current_velocity: float,
        dt: Optional[float] = None,
    ) -> float:
        """
        Compute feedforward + feedback control for trajectory following
        """
        current_time = time.time()

        # Calculate time step
        if dt is None:
            if self.previous_time is not None:
                dt = current_time - self.previous_time
            else:
                dt = 0.02  # Default 20ms

        position_error = target_position - current_position
        velocity_error = target_velocity - current_velocity

        proportional = self.kp * position_error

        self.integral += position_error * dt
        self.integral = max(
            -self.integral_limit, min(self.integral_limit, self.integral)
        )
        integral_term = self.ki * self.integral

        derivative = self.kd * velocity_error

        feedforward = target_velocity

        output = feedforward + proportional + integral_term + derivative

        output = max(-self.output_limit, min(self.output_limit, output))

        self.previous_error = position_error
        self.previous_time = current_time

        return output


class Vector2DTrajectoryController:
    """2D trajectory following controller with feedforward + feedback"""

    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.0):
        self.x_controller = PIDController(kp, ki, kd)
        self.y_controller = PIDController(kp, ki, kd)

    def update_params(self, kp: float, ki: float, kd: float):
        self.x_controller.update_params(kp, ki, kd)
        self.y_controller.update_params(kp, ki, kd)

    def reset(self):
        self.x_controller.reset()
        self.y_controller.reset()

    def compute_trajectory_following(
        self, target_state: State, current_state: State, dt: Optional[float] = None
    ) -> Vector2D:
        """
        Compute 2D trajectory following control using State objects
        """
        velocity_x = self.x_controller.compute_trajectory_following(
            target_state.position.x,
            target_state.velocity.x,
            current_state.position.x,
            current_state.velocity.x,
            dt,
        )
        velocity_y = self.y_controller.compute_trajectory_following(
            target_state.position.y,
            target_state.velocity.y,
            current_state.position.y,
            current_state.velocity.y,
            dt,
        )

        return Vector2D(velocity_x, velocity_y)


class RobotTrajectoryController:
    """Robot controller for trajectory following"""

    def __init__(self):
        self.trajectory_controllers = {}
        self.last_targets = {}  # For position triggered reset
        self.reset_threshold = 0.5  # Reset if target jumps > 0.5m

        self.default_kp = 2.3
        self.default_ki = 0.0
        self.default_kd = 0.1

    def get_controller(self, robot_id: int) -> Vector2DTrajectoryController:
        """Get or create trajectory controller for robot"""
        if robot_id not in self.trajectory_controllers:
            self.trajectory_controllers[robot_id] = Vector2DTrajectoryController(
                self.default_kp, self.default_ki, self.default_kd
            )
        return self.trajectory_controllers[robot_id]

    def compute_trajectory_command(
        self,
        robot_id: int,
        target_state: State,
        current_state: State,
        dt: Optional[float] = None,
    ) -> Vector2D:
        """
        Compute velocity command for robot to follow trajectory using State objects
        """
        controller = self.get_controller(robot_id)

        # Check for discontinuous target change
        if robot_id in self.last_targets:
            last_pos = self.last_targets[robot_id]
            target_jump = target_state.position.distance(last_pos)

            if target_jump > self.reset_threshold:
                controller.reset()

        self.last_targets[robot_id] = target_state.position

        return controller.compute_trajectory_following(target_state, current_state, dt)

    def reset_controller(self, robot_id: int):
        if robot_id in self.trajectory_controllers:
            self.trajectory_controllers[robot_id].reset()

    def update_params(self, kp: float, ki: float, kd: float):
        self.default_kp = kp
        self.default_ki = ki
        self.default_kd = kd

        for controller in self.trajectory_controllers.values():
            controller.update_params(kp, ki, kd)

    def cleanup_unused_robots(self, active_robot_ids: set):
        inactive_robots = set(self.trajectory_controllers.keys()) - active_robot_ids

        for robot_id in inactive_robots:
            if robot_id in self.trajectory_controllers:
                del self.trajectory_controllers[robot_id]
            if robot_id in self.last_targets:
                del self.last_targets[robot_id]
