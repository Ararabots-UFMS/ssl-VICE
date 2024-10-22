from movement.obstacles.interfaces import Obstacle, StaticObstacle
from movement.path.path import PathGenerator
from movement.path.path_acceptor import PathAcceptor, AcceptorStatus
from movement.path.path_profiles import MovementProfiles, DirectionProfiles
from strategy.blackboard import Blackboard
from system_interfaces.msg import Robots

from ruckig import InputParameter, OutputParameter, Result, Ruckig, Trajectory

from typing import List, Tuple


class Movement:
    def __init__(self, robot_id: int, bypass_time: float = 3):
        self.id = robot_id
        self.blackboard = Blackboard()

        self.path_generator = PathGenerator()
        self.acceptor = PathAcceptor()

        self.path_otg = Ruckig(2)
        self.orientation_otg = Ruckig(1)

    def __call__(
        self, obstacles: List[Obstacle], path_profile: MovementProfiles, orientation_profile: DirectionProfiles, sync: bool, **kwargs
    ) -> Tuple:
        # **kwargs need to be two different dicts of parameters. In this case, {path_kwargs} and {orientation_kwargs} inside kwargs.
        self.get_state()

        path_trajectory = Trajectory(2)
        orientation_trajectory = Trajectory(1)

        # Using ruckig notations
        path_inp, orientation_inp = self.path_generator.generate_input(self.get_state(), path_profile, orientation_profile, **kwargs)

        path_result = self.path_otg.calculate(path_inp, path_trajectory)
        orientation_result = self.orientation_otg.calculate(orientation_inp, orientation_trajectory)

        status, collision_obs = self.acceptor.check(path_trajectory, obstacles)

        # TODO INSIDEAREA is taking priority, it may lead to some issues
        # If inside area, any trajectory is overwriten to a trajectory to get of the area.
        if status == AcceptorStatus.INSIDEAREA:
            return self.exit_area(collision_obs)

        elif status == AcceptorStatus.ACCEPTED or (
            path_profile != MovementProfiles.Normal
            and path_profile != MovementProfiles.GetInAngle
        ): 
            if sync:
                if path_trajectory.duration < orientation_trajectory.duration:
                    path_inp.min_duration = orientation_trajectory.duration
                    path_result = self.path_otg.calculate(path_inp, path_trajectory)
                else:
                    orientation_inp.min_duration = path_trajectory.duration
                    orientation_result = self.orientation_otg.calculate(orientation_inp, orientation_trajectory)

            return path_trajectory, orientation_trajectory

        else:
            # TODO make parameters changeable.
            return self.solve_collision(obstacles, trys=5, bypass_time=3, path_profile=path_profile, **kwargs)
        

    def solve_collision(
        self, obstacles: List[Obstacle], trys: int, bypass_time: float, path_profile: MovementProfiles, **kwargs
    ):
        # Try to find a bypass trajectory and bypass orientation, if not found, break

        bypass_trajectory = Trajectory(2)
        obypass_trajectory = Trajectory(1)

        bypass_inp, obypass_inp = self.path_generator.generate_input(
            self.get_state(), MovementProfiles.Bypass, DirectionProfiles.Normal, orientation_kwargs = {'current_state': self.get_state()}
        )

        self.path_otg.calculate(bypass_inp, bypass_trajectory)

        status, obs = self.acceptor.check(bypass_trajectory, obstacles)

        if trys < 1:
            break_trajectory = Trajectory(2)
            obreak_trajectory = Trajectory(1)

            break_inp, obreak_inp = self.path_generator.generate_input(
                self.get_state(), MovementProfiles.Break, DirectionProfiles.Break
            )

            self.path_otg.calculate(break_inp, break_trajectory)
            self.path_otg.calculate(obreak_inp, obreak_trajectory)

            return break_trajectory, obreak_trajectory

        elif status == AcceptorStatus.ACCEPTED:
            new_path = Trajectory(2)

            new_inp, _ = self.path_generator.generate_input(
                bypass_trajectory.at_time(self.bypass_time)[:2], path_profile, DirectionProfiles.Break, **kwargs
            )

            self.path_otg.calculate(new_inp, new_path)

            status, obs = self.acceptor.check(new_inp, obstacles)

            if status == AcceptorStatus.ACCEPTED:
                self.path_otg.calculate(obypass_inp, obypass_trajectory)

                return bypass_trajectory, obypass_trajectory

        elif status == AcceptorStatus.COLLISION:
            return self.solve_collision(obstacles, trys - 1, bypass_time, path_profile, **kwargs)
        
        elif status == AcceptorStatus.INSIDEAREA:
            return self.exit_area(obs)

    def exit_area(self, area: StaticObstacle):
        new_trajectory = Trajectory(2)
        new_otrajectory = Trajectory(1)

        outside_point = area.closest_outside_point(self.get_state())

        # TODO: Implement a DirectionPRofile Normal and pass the parameters here...
        new_inp, new_oinp = self.path_generator.generate_input(
            self.get_state(),
            MovementProfiles.Normal,
            DirectionProfiles.Normal,
            path_kwargs = {'goal_state':(
                (outside_point[0], outside_point[1]),
            )},
            orientation_kwargs = {'current_state': self.get_state()}
        )

        self.path_otg.calculate(new_inp, new_trajectory)
        self.orientation_otg.calculate(new_oinp, new_otrajectory)

        return new_trajectory, new_otrajectory

    def get_state(self):
        robot = self.blackboard.ally_robots[self.id]

        return (
            [robot.position_x, robot.position_y, robot.orientation],
            [robot.velocity_x, robot.velocity_y, robot.velocity_orientation],
        )
