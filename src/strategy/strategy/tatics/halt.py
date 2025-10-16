from strategy.skills.skills import Skills

class HaltAction:
    def __init__(self, ally_robots):
        self.ally_robots = ally_robots
        self.skills_factory = Skills("Movement")

    def execute(self, ally_robots):
        robots_commands = []
        self.ally_robots = ally_robots

        field_ids = sorted(rid for rid in self.ally_robots.keys())

        for idx, robot_id in enumerate(field_ids):
            target_x = self.ally_robots[robot_id].position.x
            target_y = self.ally_robots[robot_id].position.y

            robot_command = self.skills_factory.move_to(
                robot_id=robot_id,
                target_x=target_x,
                target_y=target_y,
                vel_x=0.0,
                vel_y=0.0
            )
            robots_commands.append(robot_command)

        return robots_commands