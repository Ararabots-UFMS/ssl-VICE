from new_movement.entities.StaticObstacle import FieldBorderObstacle, PenaltyAreaObstacle, GenericCircleObstacle, FieldSide
from new_movement.entities.DynamicObstacles import AllyRobotObstacle, EnemyRobotObstacle
from new_movement.entities.States import State, Vector2D
from strategy.blackboard import Blackboard


class ObstacleFactory():
    def __init__(self, blackboard: Blackboard):
        self.blackboard = blackboard

    def create_obstacles(self, request, robot_data) -> list:
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
            ball = self.blackboard.balls[0] # getting the first ball, maybe revise that...
            obstacles.append(GenericCircleObstacle(Vector2D(ball.position_x, ball.position_y), 60))  # Ball radius 50mm + 10mm (safety)
        
        for enemy_id in request.enemy_ids:
            if enemy_id in self.blackboard.enemy_robots:
                robot = self.blackboard.enemy_robots[enemy_id]
                state = State(Vector2D(robot.position_x, robot.position_y), Vector2D(robot.velocity_x, robot.velocity_y))
                obstacles.append(EnemyRobotObstacle(state))
        
        for ally_id in request.ally_ids:
            if ally_id in self.blackboard.ally_robots and ally_id != request.id:
                robot = self.blackboard.ally_robots[ally_id]
                state = State(Vector2D(robot.position_x, robot.position_y), Vector2D(robot.velocity_x, robot.velocity_y))
                obstacles.append(AllyRobotObstacle(state, robot_data[ally_id]["trajectory"], radius=110))
        
        return obstacles
