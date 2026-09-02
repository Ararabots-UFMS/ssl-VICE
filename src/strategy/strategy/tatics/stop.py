from strategy.skills.skills import Skills
from utils.math_util import Vector2D
from math import cos, sin, pi, atan2


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
        robot = self.ally_robots[robot_id]
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

            angle = self._get_ball_angle(idx)

            robot_command = self.skills_factory.move_with_angle(
                robot_id=rid,
                target_x=2000,
                target_y=1400,
                angle=angle,
            )

            robot_command.ally_ids = [i for i in field_ids if i != rid]
            robot_command.field_border = True
            robot_command.penalty_area = True
            robot_command.ball = True

            robots_commands.append(robot_command)

        return robots_commands