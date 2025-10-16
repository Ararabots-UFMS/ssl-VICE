from strategy.skills.skills import Skills
from new_movement.entities.States import Vector2D


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

    def _get_valid_position(self) -> Vector2D:
        radius = 600.0  # mm

        if self.on_positive_half:
            return Vector2D(self.ball.position_x + radius, self.ball.position_y)

        return Vector2D(self.ball.position_x - radius, self.ball.position_y)

    def execute(self):
        robots_commands = []

        if 0 in self.ally_robots:
            robots_commands.append(
                GoalkeeperKickoff().execute(self.gk_target, self.angle)
            )

        field_ids = sorted([rid for rid in self.ally_robots.keys() if rid != 0])

        valid_position = self._get_valid_position()

        for idx, rid in enumerate(field_ids):
            robot_command = self.skills_factory.move_to(
                robot_id=rid,
                target_x=valid_position.x,
                target_y=valid_position.y,
            )

            robot_command.ally_ids = (id for id in field_ids if id != rid)
            robot_command.field_border = True
            robot_command.penalty_area = True
            robot_command.ball = True

            robots_commands.append(robot_command)

        return robots_commands
