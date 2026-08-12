from new_movement.entities.States import Vector2D

from strategy.skills.skills import Skills


class CenterGoal:
    GOAL_POSITIVE = Vector2D(2250.0, 0.0)
    GOAL_NEGATIVE = Vector2D(-2250.0, 0.0)


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


class OurKickoff:
    def __init__(self, ally_robots, on_positive_half):
        self.name = "OurAction"
        self.skills_factory = Skills("Movement")
        self.goal_center = CenterGoal()
        self.on_positive_half = on_positive_half
        self.ally_robots = ally_robots

        if self.on_positive_half:
            self.angle = 3.14159
            self.gk_target = self.goal_center.GOAL_POSITIVE
            self.side = 1.0
        else:
            self.angle = 0.0
            self.gk_target = self.goal_center.GOAL_NEGATIVE
            self.side = -1.0

        self.base_pos = self._get_border_circle()

    def _get_formation_offsets(self):
        """
        Retorna posições relativas (x_offset, y) dentro do campo aliado.
        - Robô 0 de linha (Chutador): Posicionado dentro do círculo central na metade aliada (200mm do centro), sem tocar a bola.
        - Demais robôs: Espalhados em 2D atrás da linha de 500mm para cobrir campo sem colidir.
        """
        return [
            (200.0, 0.0),       # Chutador (Exceção permitida no círculo central)
            (800.0, 500.0),     # Apoiador Superior
            (800.0, -500.0),    # Apoiador Inferior
            (1300.0, 800.0),    # Asa Superior
            (1300.0, -800.0),   # Asa Inferior
        ]

    def execute(self):
        robots_commands = []

        if 0 in self.ally_robots:
            robots_commands.append(
                GoalkeeperKickoff().execute(self.gk_target, self.angle))

        field_ids = sorted(
            [rid for rid in self.ally_robots.keys() if rid != 0])
        formation = self._get_formation_offsets()

        for idx, rid in enumerate(field_ids):
            if idx < len(formation):
                x_off, y_pos = formation[idx]
            else:
                x_off = 1500.0
                y_pos = 300.0 * (idx - len(formation) + 1)

            target_x = x_off * self.side
            target_y = y_pos

            robot_command = self.skills_factory.move_with_angle(
                robot_id=rid,
                target_x=target_x,
                target_y=target_y,
                vel_x=0.0,
                vel_y=0.0,
                angle=self.angle,
            )
            robot_command.field_border = True
            robot_command.center_area = True
            robot_command.penalty_area = True

            robots_commands.append(robot_command)

        return robots_commands


class TheirKickoff:
    def __init__(self, ally_robots, on_positive_half):
        self.name = "TheirAction"
        self.skills_factory = Skills("Movement")
        self.goal_center = CenterGoal()
        self.on_positive_half = on_positive_half
        self.ally_robots = ally_robots

        if self.on_positive_half:
            self.angle = 3.14159
            self.gk_target = self.goal_center.GOAL_POSITIVE
            self.side = 1.0
        else:
            self.angle = 0.0
            self.gk_target = self.goal_center.GOAL_NEGATIVE
            self.side = -1.0

        self.base_pos = self._get_border_circle()

    def _get_formation_offsets(self):
        """
        Formação defensiva recuada:
        Mantém os robôs a mais de 500mm do centro (700mm garante folga para o raio do robô).
        """
        return [
            (700.0, 0.0),       # Barreira Central (Fora do círculo de 500mm)
            (800.0, 600.0),     # Defensor Lateral Superior
            (800.0, -600.0),    # Defensor Lateral Inferior
            (1300.0, 700.0),    # Cobertura Superior
            (1300.0, -700.0),   # Cobertura Inferior
        ]

    def execute(self):
        robots_commands = []

        if 0 in self.ally_robots:
            robots_commands.append(
                GoalkeeperKickoff().execute(self.gk_target, self.angle))

        field_ids = sorted(
            [rid for rid in self.ally_robots.keys() if rid != 0])
        formation = self._get_formation_offsets()

        for idx, rid in enumerate(field_ids):
            if idx < len(formation):
                x_off, y_pos = formation[idx]
            else:
                x_off = 1500.0
                y_pos = 300.0 * (idx - len(formation) + 1)

            target_x = x_off * self.side
            target_y = y_pos

            robot_command = self.skills_factory.move_with_angle(
                robot_id=rid,
                target_x=target_x,
                target_y=target_y,
                vel_x=0.0,
                vel_y=0.0,
                angle=self.angle,
            )
            robot_command.field_border = True
            robot_command.center_area = True
            robot_command.penalty_area = True

            robots_commands.append(robot_command)

        return robots_commands