from new_movement.entities.StaticObstacle import FieldBorderObstacle, PenaltyAreaObstacle, GenericCircleObstacle, FieldSide
from new_movement.entities.DynamicObstacles import AllyRobotObstacle, EnemyRobotObstacle
from new_movement.entities.States import State, Vector2D
from new_movement.entities.Trajectory import Trajectory
from strategy.blackboard import Blackboard


class ObstacleFactory():
    def __init__(self, blackboard: Blackboard):
        self.blackboard = blackboard

    def create_obstacles(self, request, trajectories: Trajectory) -> list:
        obstacles = []
        
        if request.field_border:
            obstacles.append(FieldBorderObstacle(self.blackboard.geometry))
        
        if request.penalty_area:
            obstacles.append(PenaltyAreaObstacle(self.blackboard.geometry, FieldSide.RIGHT))
            obstacles.append(PenaltyAreaObstacle(self.blackboard.geometry, FieldSide.LEFT))
        
        if request.center_area:
            # Center Radius hardcoded
            obstacles.append(GenericCircleObstacle(Vector2D(0, 0), 500))
        
        if request.ball and self.blackboard.balls:
            ball = self.blackboard.balls[0]
            obstacles.append(GenericCircleObstacle(Vector2D(ball.x, ball.y), 60))  # Ball radius 50mm + 10mm (safety)
        
        for enemy_id in request.enemy_ids:
            if enemy_id in self.blackboard.enemy_robots:
                robot = self.blackboard.enemy_robots[enemy_id]
                state = State(Vector2D(robot.x, robot.y), Vector2D(robot.vx, robot.vy))
                obstacles.append(EnemyRobotObstacle(state))
        
        for ally_id in request.ally_ids:
            if ally_id in self.blackboard.ally_robots and ally_id != request.id:
                robot = self.blackboard.ally_robots[ally_id]
                state = State(Vector2D(robot.x, robot.y), Vector2D(robot.vx, robot.vy))
                # Using EnemyRobotObstacle for allies as well, since trajectory not available here
                obstacles.append(EnemyRobotObstacle(state))
        
        return obstacles

    

    def update_obstacles(self):
        pass