from strategy.skills.skills import Skills
from new_movement.entities.States import Vector2D
from math import cos, sin, pi, atan2


class CenterGoal:
    # 2250 -> 4500: o gol da Division B fica em x = +-4500, nao +-2250.
    #
    # 2250 e a meia-largura de um campo SSL-EL (4500 x 3000). Este projeto roda
    # em Division B: 9000 x 6000, confirmado pelas regras oficiais (sslrules.pdf
    # secao 2.1.1) e pelo proprio /game_state, que reporta campo=9000mm.
    #
    # O QUE O VALOR ERRADO CAUSAVA, e nao e sutil: o goleiro se posicionava
    # 2250 mm A FRENTE da propria meta - ou seja, abandonava o gol e parava
    # perto do meio-campo - e os atacantes miravam um ponto vazio no meio do
    # campo adversario. Em jogo aberto o time inteiro converge para o centro.
    #
    # A mesma constante ja existia errada em tatics/freekick.py e foi corrigida
    # la ha tempos; kickoff.py, stop.py e running.py ficaram para tras (o
    # HANDOVER §6.2 registra as tres como "nao corrigidas"). Esta e a correcao
    # que faltava.
    GOAL_POSITIVE = Vector2D(4500.0, 0.0)
    GOAL_NEGATIVE = Vector2D(-4500.0, 0.0)


class GoalkeeperKickoff:
    def __init__(self):
        self.name = "GoalkeeperKickoff"
        self.skills_factory = Skills("Movement")

    def execute(self, goal_position: Vector2D, angle: float):
        robot_command = self.skills_factory.move_with_angle(
            robot_id=0,
            target_x=goal_position.x,
            target_y=goal_position.y,
            vel_x=0.0,
            vel_y=0.0,
            angle=angle,
        )
        robot_command.field_border = True
        return robot_command


class goAwayFromBall:
    def __init__(self, ally_robots, ball, on_positive_half):
        self.name = "goAwayFromBall"
        self.skills_factory = Skills("Movement")
        self.ally_robots = ally_robots
        self.ball = ball
        self.on_positive_half = on_positive_half
        self.goal_center = CenterGoal()

        if self.on_positive_half:
            self.angle = 3.14159
            self.gk_target = self.goal_center.GOAL_POSITIVE
        else:
            self.angle = 0.0
            self.gk_target = self.goal_center.GOAL_NEGATIVE

    def _generate_positions(self, count: int) -> list[Vector2D]:
        """
        Generate 'count' positions around the ball on a safe circle.
        Uses a base angle pointing away/toward our half to keep robots on the safer side.
        """
        # keep >= 500mm from ball. Using a bit more for safety.
        radius = 600.0
        bx, by = self.ball.position_x, self.ball.position_y

        # Base angle: if we are on positive half (attacking +x), place first point toward +x, else -x
        base_angle = 0.0 if self.on_positive_half else pi

        # Evenly spaced around the circle
        step = (2 * pi) / max(count, 1)

        positions: list[Vector2D] = []
        for i in range(count):
            ang = base_angle + i * step
            x = bx + radius * cos(ang)
            y = by + radius * sin(ang)
            positions.append(Vector2D(x, y))

        return positions

    def _get_ball_angle(self, robot_id: int) -> float:
        # .get() em vez de indexacao direta: se o robo nao estiver em campo,
        # devolvemos 0.0 em vez de derrubar o node inteiro com KeyError.
        robot = self.ally_robots.get(robot_id)
        if robot is None:
            return 0.0
        dx = self.ball.position_x - robot.position_x
        dy = self.ball.position_y - robot.position_y
        return atan2(dy, dx)

    def execute(self):
        robots_commands = []

        if 0 in self.ally_robots:
            robots_commands.append(
                GoalkeeperKickoff().execute(self.gk_target, self.angle)
            )

        field_ids = sorted([rid for rid in self.ally_robots.keys() if rid != 0])

        targets = self._generate_positions(len(field_ids))

        for idx, rid in enumerate(field_ids):
            target = targets[idx]

            # 'rid', nao 'idx'.
            #
            # Passava o indice do enumerate (0, 1, 2...) no lugar do id do robo.
            # Com o robo 0 ausente virava ally_robots[0] -> KeyError, e o
            # strategyNode MORRIA - nenhuma jogada rodava, nem o freekick. Com o
            # robo 0 presente nao quebrava, mas calculava o angulo do robo errado.
            angle = self._get_ball_angle(rid)

            # USA O ALVO CALCULADO, nao um ponto fixo.
            #
            # _generate_positions distribui os robos num circulo de 600 mm em
            # torno da bola - que e o que a regra do STOP pede (ninguem a menos
            # de 500 mm). O resultado era calculado na linha acima e DESCARTADO:
            # todo mundo recebia (2000, 1400).
            #
            # Consequencia: durante qualquer STOP o time inteiro converge para
            # UM ponto no campo de ataque, longe da bola. Medido nesta sessao no
            # cenario 'passe' - o cobrador nascia em (852,976) em vez de
            # (-400,0) e o receptor em (1996,1366), os dois a caminho de
            # (2000,1400), e a cobranca comecava com o time desmanchado.
            #
            # O HANDOVER §6.5 ja registrava isto como "outro problema no mesmo
            # metodo, NAO corrigido".
            robot_command = self.skills_factory.move_with_angle(
                robot_id=rid,
                target_x=target.x,
                target_y=target.y,
                angle=angle,
            )

            robot_command.ally_ids = [i for i in field_ids if i != rid]
            robot_command.field_border = True
            robot_command.penalty_area = True
            robot_command.ball = True

            robots_commands.append(robot_command)

        return robots_commands