from entities.Trajectory import TrajectorySegment
from entities.States import State, Vector2D
from entities.obstacles import Obstacle
from utilities.trajectory_generator.TrajGenerator import TrajectoryGenerator

from strategy.blackboard import Blackboard

from copy import deepcopy
from typing import List, Optional
from random import uniform, randint


class TrajectoryOptimizer():
    def __init__(self):
        pass

    def optimize(self, trajectory: TrajectorySegment, generator: TrajectoryGenerator) -> TrajectorySegment:
        trajectory = deepcopy(trajectory) # Dont know if necessary
        total_time = trajectory.get_total_time()

        if total_time <= 0:
            return trajectory
        
        mode = ["Head", "Normal", "Tail"][randint(0, 2)]

        first_time = uniform(0, total_time)

        if(mode == "Head"):
            curState = trajectory.get_state(0.0)
            tarState = trajectory.get_state(first_time)

            return generator.generate(curState, tarState), 0.0, first_time
        
        elif(mode == "Tail"):
            curState = trajectory.get_state(first_time)
            tarState = trajectory.get_destination()

            return generator.generate(curState, tarState), first_time, total_time
        
        elif(mode == "Normal"):
            second_time = uniform(first_time, total_time)
            curState = trajectory.get_state(first_time)
            tarState = trajectory.get_state(second_time)

            return generator.generate(curState, tarState), first_time, second_time

class CollisionSolver():
    def __init__(self, trys: int = 50):
        self.trys = trys
        self.blackboard = Blackboard()
    
    def solve(self, curState: State, tarState: State, obstacles: List[Obstacle], generator: TrajectoryGenerator) -> TrajectorySegment:
        ''' Finds a new trajectory without collisions based on a random sampling method similar to RRT '''
        best_trajectory = None
        for i in range(self.trys):
            # random point to attemp bypass
            random_point: Vector2D = self.generate_random_point()
            bypassState = State(random_point, Vector2D(0, 0))

            to_point: TrajectorySegment = generator(curState, bypassState)
            from_point: TrajectorySegment = generator(bypassState, tarState)

            to_point.add_child(from_point)

            if self.is_collision(to_point, obstacles) is True:
                continue
            else:
                # TODO Run points systems to compare trajectorys found
                best_trajectory = to_point

        return best_trajectory
                
    def is_collision(self, trajectory: TrajectorySegment, obstacles: List[Obstacle]) -> bool:
        ''' Checks if a trajectory has collision with any obstacles given, returns True is theres is, otherwise returns False '''
        for obs in obstacles:
            if obs.collisionAt(trajectory) is None:
                return True

        return False

    def generate_random_point(self) -> Vector2D:
        ''' Generates a random point inside game field '''
        field_length = self.blackboard.geometry.field_length
        field_width = self.blackboard.geometry.field_width

        return Vector2D(uniform(-field_length/2, field_length/2), uniform(-field_width/2, field_width/2))

class Planner():
    ''' Its responsible for finding a collision free optimal trajectory to target state '''
    def __init__(self, bypass_trys: Optional[int] = None, optimizer_trys: Optional[int] = 50):
        self.generator = TrajectoryGenerator()
        self.solver = CollisionSolver() if bypass_trys is None else CollisionSolver(trys=bypass_trys)
        self.optimizer = TrajectoryOptimizer()
        self.optimizer_trys = optimizer_trys

    def find(self, curState: State, tarState: State, obstacles: List[Obstacle]) -> TrajectorySegment:
        best_trajectory: TrajectorySegment = self.generator.generate(curState, tarState)

        if(self.solver.is_collision(best_trajectory, obstacles)):
            best_trajectory = self.solver.solve(curState, tarState, obstacles, self.generator)
            for _ in range(self.optimizer_trys):
                opt_segment, first_time, second_time = self.optimizer.optimize(best_trajectory)
                if not self.solver.is_collision(opt_segment):
                    # ISSO É TALVEZ NAO SEJA LEGAL ESTAR AQUI, TALVEZ ENCAIXAR ESSA PARTE EM TRAJECTORY.py
                    # AQUI TA SEPARANDO O CAMINHO EM (PRIMEIRA PARTE) + (PARTE QUE VAI SER SUBSTITUIDA) + (SEGUNDA PARTE)
                    _, second_segment = best_trajectory.motionPath.split(second_time)
                    first_segment, _ = best_trajectory.motionPath.split(first_time)
                    best_trajectory.motionPath = first_segment.motion_path + opt_segment.motionPath.motion_path + second_segment.motion_path
        else:
            return best_trajectory
        
from utilities.trajectory_plotter import TrajectoryPlotter
from entities.Trajectory import Trajectory

plotter = TrajectoryPlotter()

initState = State(Vector2D(0, 0), Vector2D(0, 0))
tarState = State(Vector2D(100, 100), Vector2D(100, 100))

planner = Planner()
trajseg = planner.find(initState, tarState, [])
traj = Trajectory(trajseg)

plotter.plot(traj)