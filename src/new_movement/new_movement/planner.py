from new_movement.entities.Trajectory import TrajectorySegment, Trajectory
from new_movement.entities.States import State, Vector2D
from new_movement.entities.obstacles import Obstacle
from new_movement.utilities.trajectory_generator.TrajGenerator import TrajectoryGenerator

from strategy.blackboard import Blackboard

from typing import List, Optional
from random import uniform, randint


class CollisionSolver():
    def __init__(self, trys: int = 50):
        self.trys = trys
        self.blackboard = Blackboard()
    
    def solve(self, curState: State, tarState: State, obstacles: List[Obstacle], generator: TrajectoryGenerator) -> TrajectorySegment:
        ''' Finds a new trajectory without collisions based on a random sampling method similar to RRT '''
        best_trajectory = None
        for i in range(self.trys):
            # random point to attempt bypass
            random_point: Vector2D = self.generate_random_point()
            bypassState = State(random_point, Vector2D(0, 0))

            to_point: TrajectorySegment = generator(curState, bypassState)
            from_point: TrajectorySegment = generator(bypassState, tarState)

            to_point.add_child(from_point)

            # TODO Define what is the best trajectory.
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

class TrajectoryOptimizer():
    def __init__(self, trys: int):
        self.trys = trys

    def optimize(self, trajectory: Trajectory, generator: TrajectoryGenerator, collisionSolver: CollisionSolver, obstacles: List[Obstacle]) -> Trajectory:
        """ 
        Optimizes a trajectory, by random sampling a range in the trajectory and finding a new collision free segment 
        
        How it works:
            - A random mode is chosen, either head, normal or tail
            - if the mode is normal, then 2 points are randomly chosen and a new path is generated from point 1 to point 2
            - if the mode is head or tail, one point is randomly chosen and a new path is generated from the begining 
            to the point (in head mode) or the point to the ending (tail mode)
            - a collision check is made, and if no collision is detected, then this new patch is stiched. 

        This approach is discribed in the article "Bang Bang Boosting of RRTs" 
        
        """
        total_time = trajectory.get_total_duration()

        if total_time <= 0:
            return trajectory
        
        for _ in range(self.trys):

            mode = ["Head", "Normal", "Tail"][randint(0, 2)]

            first_time = uniform(0, total_time) if (mode == "Normal" or mode == "Tail") else 0.0
            second_time = uniform(first_time, total_time) if (mode == "Normal" or mode == "Head") else total_time
            
            firstState = trajectory.get_state(first_time)
            secondState = trajectory.get_state(second_time)

            optimized_segment = generator.generate(firstState, secondState)

            if(not collisionSolver.is_collision(optimized_segment, obstacles)):
                curSegment = trajectory.root
                curTime = second_time
                while(curSegment.get_local_duration() < curTime and curSegment.child is not None):
                    curTime -= curSegment.get_local_duration()
                    curSegment = curSegment.child
                
                _, last_motion = curSegment.motion_path.split(curTime)
                last_segment = TrajectorySegment(secondState.position, secondState.velocity, last_motion)
                
                if(curSegment.child is not None):
                    last_segment.child = curSegment.child

                optimized_segment.add_child(last_segment)
                trajectory.connect(optimized_segment, first_time)

                total_time = trajectory.get_total_duration()

        return trajectory

class Planner():
    ''' Its responsible for finding a collision free optimal trajectory to target state '''
    def __init__(self, bypass_trys: Optional[int] = None, optimizer_trys: Optional[int] = 50):
        self.generator = TrajectoryGenerator()
        self.solver = CollisionSolver() if bypass_trys is None else CollisionSolver(trys=bypass_trys)
        self.optimizer = TrajectoryOptimizer(optimizer_trys)
        self.optimizer_trys = optimizer_trys

    def find(self, curState: State, tarState: State, obstacles: List[Obstacle]) -> TrajectorySegment:
        best_trajectory: TrajectorySegment = self.generator.generate(curState, tarState)

        if(self.solver.is_collision(best_trajectory, obstacles)):
            best_trajectory = self.solver.solve(curState, tarState, obstacles, self.generator)
            self.optimizer.optimize(best_trajectory, self.generator, self.solver, obstacles)

        return best_trajectory