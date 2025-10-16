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
        self.padding = 500.0  # mm
        self.skills_factory = Skills("Movement")
        self.goal_center = CenterGoal()
        self.on_positive_half = on_positive_half
        self.ally_robots = ally_robots

        if self.on_positive_half:
            self.angle = 3.14159
            self.gk_target = self.goal_center.GOAL_POSITIVE
        else:
            self.angle = 0.0
            self.gk_target = self.goal_center.GOAL_NEGATIVE

        self.base_pos = self._get_border_circle()

    def _get_border_circle(self) -> Vector2D:
        radius = 500
        if self.on_positive_half:
            return Vector2D(radius, 0.0)
        return Vector2D(-radius, 0.0)

    def execute(self):
        robots_commands = []

        if 0 in self.ally_robots:
            robots_commands.append(GoalkeeperKickoff().execute(self.gk_target, self.angle))

        field_ids = sorted([rid for rid in self.ally_robots.keys() if rid != 0])

        direction = 1.0 if self.base_pos.x >= 0 else -1.0

        for idx, rid in enumerate(field_ids):
            target_x = self.base_pos.x if idx == 0 else self.base_pos.x + direction * idx * self.padding
            target_y = self.base_pos.y

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
        self.padding = 500.0  # mm
        self.skills_factory = Skills("Movement")
        self.goal_center = CenterGoal()
        self.on_positive_half = on_positive_half
        self.ally_robots = ally_robots

        if self.on_positive_half:
            self.angle = 3.14159
            self.gk_target = self.goal_center.GOAL_POSITIVE
        else:
            self.angle = 0.0
            self.gk_target = self.goal_center.GOAL_NEGATIVE

        self.base_pos = self._get_border_circle()

    def _get_border_circle(self) -> Vector2D:
        radius = 500
        if self.on_positive_half:
            return Vector2D(radius, 0.0)
        return Vector2D(-radius, 0.0)

    def execute(self):
        robots_commands = []

        if 0 in self.ally_robots:
            robots_commands.append(GoalkeeperKickoff().execute(self.gk_target, self.angle))

        field_ids = sorted([rid for rid in self.ally_robots.keys() if rid != 0])

        direction = 1.0 if self.base_pos.x >= 0 else -1.0

        for idx, rid in enumerate(field_ids):
            target_x = self.base_pos.x if idx == 0 else self.base_pos.x + direction * idx * self.padding
            target_y = self.base_pos.y

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