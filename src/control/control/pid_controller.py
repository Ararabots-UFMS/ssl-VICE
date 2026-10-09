import time
from typing import Optional

from movement.entities.motion import MotionState

from utils.math_util import Vector2D

DEFAULT_KP = 2.3
DEFAULT_KI = 0.0
DEFAULT_KD = 0.1
DEFAULT_SLEW_LIMIT = 3.0  # m/s²


class PIDController:
    def __init__(self, kp: float, ki: float, kd: float, slew_limit: float = 3.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.previous_error: Optional[float] = None
        self.integral: float = 0.0
        self.previous_time: Optional[float] = None
        self.previous_output: Optional[float] = None

        self.integral_limit: float = 1  # Anti Windup
        self.output_limit: float = 3  # Max velocity
        # m/s². Should match what the robot can actually deliver: a command that steps
        # faster than that is answered with wheel slip, not acceleration.
        self.slew_limit: float = slew_limit

    def update_params(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def reset(self):
        self.previous_error = None
        self.integral = 0.0
        self.previous_time = None
        self.previous_output = None

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

        # O FEEDFORWARD VALE SO ONDE CONCORDA COM O ERRO DE POSICAO.
        #
        # Isto ja foi um ajuste de teste (bloco ARARABOTS_AJUSTE, ligado pelo
        # ararabots.sh). Virou codigo em 07/10/2026, porque as duas versoes que
        # ele alternava estao medidas e as duas estao erradas.
        #
        # 1. FEEDFORWARD CRU: DOMINA E APONTA PARA O LUGAR ERRADO.
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
        # de onde o erro de posicao manda. Ele foge do alvo. Magnitudes na mesma
        # faixa, o que explica a dominancia: velocidade do setpoint com p90 de
        # 223-451 mm/s e maximo de 695, contra kp*erro = 1,5 * 0,3 m = 450 mm/s.
        #
        # 2. SEM FEEDFORWARD: O ROBO NAO EMPURRA MAIS NADA.
        #
        # Foi a correcao de 25/08 (A/B de 4 contra 12 execucoes: yy de 82,0 para
        # 28,5 mm de mediana), e ela custou a velocidade inteira. Sem o termo
        # sobra 'kp * erro_de_posicao' - e o erro contra o REFERENCIAL DO
        # RASTREADOR e minusculo por construcao: ele emite um ponto logo a frente
        # do robo e replaneja a cada ciclo.
        #
        # MEDIDO no replay de 'empurrao_reto' (07/10/2026), robo em contato com
        # a bola de t = 9,6 s ao fim da execucao:
        #     distancia robo -> setpoint do rastreador   4 a 50 mm (mediana ~35)
        #     kp * erro  =  2,3 * 0,035                  = 0,08 m/s
        #     a bola andou                               829 mm em 24,5 s
        #     o robo passou 71% da execucao EM CONTATO com a bola
        #
        # O alvo da estrategia pode estar 291 mm adiante (skills/aproximacao.py,
        # EMPURRAO) e nao muda nada, porque quem fala com o PID e o rastreador,
        # nao a estrategia. O canal que carrega a velocidade PLANEJADA e
        # justamente o feedforward.
        #
        # 3. A GUARDA: mesmo termo, com uma comparacao de sinais.
        #
        # Mantem o feedforward apenas quando ele empurra para o mesmo lado que o
        # erro de posicao. Com os ~140 graus medidos, o eixo que aponta para o
        # lado errado e zerado e o outro sobrevive. Nao e ganho novo nem filtro
        # novo.
        #
        # MEDIDO, mesma execucao de 'empurrao_reto', mesma geometria:
        #                                sem feedforward     com a guarda
        #     avanco da bola                      829 mm        4543 mm
        #     chute                         nao disparou   5708 mm/s
        #     tempo em contato                       71%             1%
        #     erro de rastreio (med/p90)       26/136 mm      9/70 mm
        #     yy na maior aproximacao             110 mm          15 mm
        #                                                  (limite grSim: 40)
        #
        # Por que por eixo e nao vetorialmente: este controlador E escalar - o
        # Vector2DTrajectoryController chama x e y separadamente, cada um com o
        # seu integrador. Projetar no versor do erro exigiria mover a conta para
        # o nivel 2D, e isso muda a estrutura de um pacote que nao e nosso.
        #
        # O QUE AINDA ESTA ERRADO, e nao e aqui: o rastreador replaneja a cada
        # ciclo a partir do estado planejado. Esta guarda e remendo no
        # consumidor; a correcao esta no produtor (pacote movement).
        if feedforward * position_error > 0.0:
            output = feedforward + proportional + integral_term + derivative
        else:
            output = proportional + integral_term + derivative

        output = max(-self.output_limit, min(self.output_limit, output))

        # A reference discontinuity — a replan, a handoff, the goal moving — would
        # otherwise reach the wheels as a step. Seeded from the measurement so a fresh
        # controller resumes from where the robot actually is.
        if self.previous_output is None:
            self.previous_output = current_velocity

        if dt > 0:
            max_delta = self.slew_limit * dt
            output = max(
                self.previous_output - max_delta,
                min(self.previous_output + max_delta, output),
            )

        self.previous_error = position_error
        self.previous_output = output

        return output


class Vector2DTrajectoryController:
    """2D trajectory following controller with feedforward + feedback"""

    def __init__(
        self,
        kp: float = DEFAULT_KP,
        ki: float = DEFAULT_KI,
        kd: float = DEFAULT_KD,
        slew_limit: float = DEFAULT_SLEW_LIMIT,
    ):
        self.x_controller = PIDController(kp, ki, kd, slew_limit)
        self.y_controller = PIDController(kp, ki, kd, slew_limit)

    def update_params(self, kp: float, ki: float, kd: float):
        self.x_controller.update_params(kp, ki, kd)
        self.y_controller.update_params(kp, ki, kd)

    def reset(self):
        self.x_controller.reset()
        self.y_controller.reset()

    def compute_trajectory_following(
        self, target_state: MotionState, current_state: MotionState, dt: Optional[float] = None
    ) -> Vector2D:
        """
        Compute 2D trajectory following control using MotionState objects
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

        self.default_kp = DEFAULT_KP
        self.default_ki = DEFAULT_KI
        self.default_kd = DEFAULT_KD
        self.default_slew_limit = DEFAULT_SLEW_LIMIT

    def get_controller(self, robot_id: int) -> Vector2DTrajectoryController:
        """Get or create trajectory controller for robot"""
        if robot_id not in self.trajectory_controllers:
            self.trajectory_controllers[robot_id] = Vector2DTrajectoryController(
                self.default_kp,
                self.default_ki,
                self.default_kd,
                self.default_slew_limit,
            )
        return self.trajectory_controllers[robot_id]

    def compute_trajectory_command(
        self,
        robot_id: int,
        target_state: MotionState,
        current_state: MotionState,
        dt: Optional[float] = None,
    ) -> Vector2D:
        """
        Compute velocity command for robot to follow trajectory using MotionState objects
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
