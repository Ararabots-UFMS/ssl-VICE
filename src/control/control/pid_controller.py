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

        self.integral_limit: float = 1  # Anti Windup
        self.output_limit: float = 3  # Max velocity

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
        if dt is None:
            current_time = time.time()
            dt = (current_time - self.previous_time) if self.previous_time else 0.02
            self.previous_time = current_time

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

        # >>> ARARABOTS_AJUSTE   (ligado/desligado por: ararabots.sh ajustes on|off controle)
        #
        # O FEEDFORWARD DOMINA E APONTA PARA O LUGAR ERRADO.
        #
        # 'target_velocity' vem de trajectory.get_state(time_offset) no driver.
        # Como replan() zera o time_offset a cada chamada (driver.py:331) e
        # planeja a partir do estado PLANEJADO (nao do medido), o que volta e a
        # velocidade que o robo tinha quando o plano foi feito. O feedforward
        # passa a mandar "continue fazendo o que estava fazendo", e isso nunca
        # se corrige: e realimentacao positiva.
        #
        # MEDIDO em duas execucoes, comparando a direcao em que o robo ANDA com:
        #     a velocidade do setpoint (feedforward):  49 graus e 43 graus
        #     o erro de posicao:                      145 graus e 136 graus
        #
        # Ou seja: o robo acompanha o feedforward e anda ~140 graus AO CONTRARIO
        # de onde o erro de posicao manda. Ele foge do alvo.
        #
        # Magnitudes na mesma faixa, o que explica a dominancia: velocidade do
        # setpoint com p90 de 223-451 mm/s e maximo de 695, contra kp*erro =
        # 1,5 * 0,3 m = 450 mm/s. Com o erro pequeno, o feedforward vence.
        #
        # Sem o feedforward sobra a malha fechada na posicao medida, que e o que
        # se quer aqui. O termo faz sentido quando a trajetoria e confiavel -
        # nao e o caso enquanto o time_offset zerar a cada ciclo.
        #AJUSTE# output = proportional + integral_term + derivative
        output = feedforward + proportional + integral_term + derivative
        # <<< ARARABOTS_AJUSTE

        output = max(-self.output_limit, min(self.output_limit, output))

        self.previous_error = position_error

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

        self.default_kp = 1
        self.default_ki = 0.0
        self.default_kd = 0.0

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
