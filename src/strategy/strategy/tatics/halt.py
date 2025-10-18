from strategy.skills.skills import Skills


class HaltAction:
    def __init__(self, ally_robots):
        self.skills_factory = Skills("Movement")
        self.ally_robots = ally_robots

    def execute(self):
        robots_commands = []

        for robot_id, robot_info in self.ally_robots.items():
            target_x = robot_info.position_x
            target_y = robot_info.position_y

            robot_command = self.skills_factory.move_to(
                robot_id=robot_id,
                target_x=target_x,
                target_y=target_y,
                vel_x=0.0,
                vel_y=0.0,
            )

            robots_commands.append(robot_command)

        return robots_commands
