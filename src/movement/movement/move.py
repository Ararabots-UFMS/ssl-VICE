from movement.obstacles.interfaces import Obstacle, StaticObstacle
from system_interfaces.msg import Robots
from strategy.blackboard import Blackboard
from movement.path.path import PathGenerator
from movement.path.path_acceptor import PathAcceptor, AcceptorStatus
from movement.path.path_profiles import MovementProfiles

from ruckig import InputParameter, OutputParameter, Result, Ruckig, Trajectory

from typing import List, Tuple


class Movement:
    def __init__(self, robot_id: int):
        self.id = robot_id
        self.blackboard = Blackboard()

        self.path_generator = PathGenerator()
        self.acceptor = PathAcceptor()

        self.otg = Ruckig(3)

    def __call__(
        self, obstacles: List[Obstacle], profile: MovementProfiles, **kwargs
    ) -> Tuple:
        self.get_state()

        trajectory = Trajectory(3)

        # Using ruckig notations
        inp = self.path_generator.generate_input(self.get_state(), profile, **kwargs)

        result = self.otg.calculate(inp, trajectory)

        status, collision_obs = self.acceptor.check(trajectory, obstacles)

        # TODO INSIDEAREA is taking priority, it may lead to some issues
        # If inside area, any trajectory is overwriten to a trajectory to get of the area.
        if status == AcceptorStatus.INSIDEAREA:
            return self.exit_area(collision_obs)

        elif status == AcceptorStatus.ACCEPTED or (
            profile != MovementProfiles.Normal
            and profile != MovementProfiles.GetInAngle
        ):
            return trajectory, None

        else:
            return self.solve_collision(obstacles, trys=5, bypass_time=10, **kwargs)

    def solve_collision(
        self, obstacles: List[Obstacle], trys: int, bypass_time: float, **kwargs
    ):
        self.update_state()

        # Try to find a bypass trajectory, if not found, break
        for _ in range(trys):
            bypass_trajectory = Trajectory(3)
            bypass_inp = self.path_generator.generate_input(
                self.get_state, MovementProfiles.Bypass
            )

            self.otg.calculate(bypass_inp, bypass_trajectory)

            # Found bypass without collision, now verify if can reach target state collision free too.
            bypass_status, _ = self.acceptor.check(bypass_trajectory, obstacles)
            if bypass_status == AcceptorStatus.ACCEPTED:
                bypass_pos, bypass_vel, _ = bypass_trajectory.at_time(bypass_time)

                new_trajectory = Trajectory(3)
                new_inp = self.path_generator.generate_input(
                    (bypass_pos, bypass_vel), kwargs["goal_state"]
                )

                self.otg.calculate(new_inp, new_trajectory)

                new_status, _ = self.acceptor.check(new_trajectory, obstacles)
                if new_status == AcceptorStatus.ACCEPTED:
                    return bypass_trajectory, new_trajectory

        # No path was found, breaking
        # TODO Don't know if breaking is the best action to take if no path is found
        break_trajectory = Trajectory(3)
        break_inp = self.path_generator.generate_input(
            self.get_state, MovementProfiles.Break
        )

        self.otg.calculate(break_inp, break_trajectory)

        return break_trajectory, None

    def exit_area(self, area: StaticObstacle):
        self.update_state()

        new_trajectory = Trajectory(3)
        outside_point = area.closest_outside_point(self.get_state)
        new_inp = self.path_generator.generate_input(
            self.get_state,
            MovementProfiles.Normal,
            goal_state=(
                [outside_point[0], outside_point[1], self.get_state[0][2]],
                [0, 0, 0],
            ),
        )

        self.otg.calculate(new_inp, new_trajectory)

        return new_trajectory, None

    def get_state(self):
        robot = self.blackboard.ally_robots[self.id]

        return (
            [robot.position_x, robot.position_y, robot.orientation],
            [robot.velocity_x, robot.velocity_y, robot.velocity_orientation],
        )
