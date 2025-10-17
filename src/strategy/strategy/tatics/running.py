from new_movement.entities.States import Vector2D
from math import atan2, hypot

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


class Atack:
    def __init__(self, ally_robots, ball, on_positive_half):
        self.name = "OurAtack"
        self.skills_factory = Skills("Movement")
        self.goal_center = CenterGoal()
        self.on_positive_half = on_positive_half
        self.ally_robots = ally_robots
        self.ball = ball
        self.kick_threshold = 1125.0

        if self.on_positive_half:
            self.gk_angle = 3.14159
            self.gk_target = self.goal_center.GOAL_POSITIVE
            self.attack_goal = self.goal_center.GOAL_NEGATIVE
        else:
            self.gk_angle = 0.0
            self.gk_target = self.goal_center.GOAL_NEGATIVE
            self.attack_goal = self.goal_center.GOAL_POSITIVE

    def _can_kick(self):
        if self.on_positive_half and self.ball.position_x < -self.kick_threshold:
            return True
        elif not self.on_positive_half and self.ball.position_x > self.kick_threshold:
            return True

        return False

    def _get_angle_to_goal(self, robot_id) -> float:
        robot = None
        for rid, robot_info in self.ally_robots.items():
            if rid == robot_id:
                robot = robot_info
                break

        goal_pos = self.attack_goal

        if robot is None:
            return 0.0

        dx = goal_pos.x - robot.position_x
        dy = goal_pos.y - robot.position_y

        angle = atan2(dy, dx)
        return angle

    def _get_angle_to_ball(self, robot_id) -> float:
        robot = None
        for rid, robot_info in self.ally_robots.items():
            if rid == robot_id:
                robot = robot_info
                break

        if robot is None:
            return 0.0

        dx = self.ball.position_x - robot.position_x
        dy = self.ball.position_y - robot.position_y

        angle = atan2(dy, dx)
        return angle

    def _go_to_ball(self, robot_id):
        angle = self._get_angle_to_ball(robot_id)

        robot_command = self.skills_factory.move_with_angle(
            robot_id=robot_id,
            target_x=self.ball.position_x,
            target_y=self.ball.position_y,
            vel_x=0.0,
            vel_y=0.0,
            angle=angle,
        )

        robot_command.field_border = True
        robot_command.ball = True
        robot_command.penalty_area = True

        return robot_command

    def _go_to_goal(self, robot_id):
        """
        Go to a point behind the ball (relative to the opponent goal) and push through it.
        """
        bx, by = self.ball.position_x, self.ball.position_y
        gx, gy = self.attack_goal.x, self.attack_goal.y
        dx, dy = gx - bx, gy - by
        norm = hypot(dx, dy) or 1.0
        ux, uy = dx / norm, dy / norm

        behind_dist = 250.0
        stage_x = bx - ux * behind_dist
        stage_y = by - uy * behind_dist

        # Choose stage vs push target
        rx = ry = None
        for rid, robot_info in self.ally_robots.items():
            if rid == robot_id:
                rx, ry = robot_info.position_x, robot_info.position_y
                break

        target_x, target_y = stage_x, stage_y
        if rx is not None:
            dist_to_stage = hypot(rx - stage_x, ry - stage_y)
            dist_to_ball = hypot(rx - bx, ry - by)
            if dist_to_stage < 120.0 or dist_to_ball < 180.0:
                target_x = bx + ux * 220.0
                target_y = by + uy * 220.0

        # Orient along ball->goal line
        angle = atan2(dy, dx)

        robot_command = self.skills_factory.move_with_angle(
            robot_id=robot_id,
            target_x=target_x,
            target_y=target_y,
            vel_x=0.0,
            vel_y=0.0,
            angle=angle,
        )

        robot_command.field_border = True
        robot_command.ball = False
        robot_command.penalty_area = True

        robot_can_kick = self._can_kick()

        if robot_can_kick:
            robot_command.activate_kick()
        else:
            robot_command.deactivate_kick()

        return robot_command

    def _robot_close_to_ball(self, robot_id) -> bool:
        distance = 200

        ball_pos = Vector2D(self.ball.position_x, self.ball.position_y)

        robot_pos = None
        for robot_id_, robot_info in self.ally_robots.items():
            if robot_id_ == robot_id:
                robot_pos = Vector2D(robot_info.position_x, robot_info.position_y)
                break

        if robot_pos is not None and robot_pos.distance(ball_pos) < distance:
            return True

        return False

    def _robot_is_stable(self, robot_id) -> bool:
        for robot_id_, robot_info in self.ally_robots.items():
            if robot_id_ == robot_id:
                if (
                    abs(robot_info.velocity_x) < 25
                    and abs(robot_info.velocity_y) < 25
                ):
                    return True
                break

        return False

    def execute(self):
        robots_commands = []

        if 0 in self.ally_robots:
            robots_commands.append(
                GoalkeeperKickoff().execute(self.gk_target, self.gk_angle)
            )

        for robot_id_, _ in self.ally_robots.items():
            if robot_id_ == 0:
                continue

            robot_close_to_ball = self._robot_close_to_ball(robot_id_)

            if robot_close_to_ball:
                robots_commands.append(self._go_to_goal(robot_id=robot_id_))
            else:
                robots_commands.append(self._go_to_ball(robot_id=robot_id_))

        return robots_commands


class Defense:
    def __init__(self, ally_robots, ball, on_positive_half):
        self.name = "OurDefense"
        self.skills_factory = Skills("Movement")
        self.goal_center = CenterGoal()
        self.on_positive_half = on_positive_half
        self.ally_robots = ally_robots
        self.ball = ball

        if self.on_positive_half:
            self.angle = 3.14159
            self.gk_target = self.goal_center.GOAL_POSITIVE
        else:
            self.angle = 0.0
            self.gk_target = self.goal_center.GOAL_NEGATIVE

    def _go_to_ball(self) -> Vector2D:
        pass

    def execute(self):
        robots_commands = []

        if 0 in self.ally_robots:
            robots_commands.append(
                GoalkeeperKickoff().execute(self.gk_target, self.angle)
            )

        for robot_id_, _ in self.ally_robots.items():
            if robot_id_ == 0:
                continue

            ball_pos = Vector2D(self.ball.position_x, self.ball.position_y)
            goal_pos = (
                self.goal_center.GOAL_POSITIVE
                if self.on_positive_half
                else self.goal_center.GOAL_NEGATIVE
            )
            defend_x = (ball_pos.x + goal_pos.x) / 2.0
            defend_y = (ball_pos.y + goal_pos.y) / 2.0

            robot_command = self.skills_factory.move_with_angle(
                robot_id=robot_id_,
                target_x=defend_x,
                target_y=defend_y,
                vel_x=0.0,
                vel_y=0.0,
                angle=self.angle,
            )

            robot_command.field_border = True
            robot_command.ball = False
            robot_command.penalty_area = True

            robots_commands.append(robot_command)

        return robots_commands
