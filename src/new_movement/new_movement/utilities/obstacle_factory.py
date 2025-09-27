from new_movement.entities.StaticObstacle import (
    FieldBorderObstacle,
    PenaltyAreaObstacle,
    GenericCircleObstacle,
    FieldSide,
)
from new_movement.entities.DynamicObstacles import AllyRobotObstacle, EnemyRobotObstacle
from new_movement.entities.States import State, Vector2D


class ObstacleFactory:
    def __init__(self, game_watcher=None):
        self.game_watcher = game_watcher

    def create_obstacles(self, request, robot_data) -> list:
        obstacles = []

        # Get data from game_watcher if available
        if self.game_watcher:
            geometry = self.game_watcher.get_geometry()
            balls = self.game_watcher.get_balls()
            enemy_robots = self.game_watcher.get_enemy_robots()
            ally_robots = self.game_watcher.get_ally_robots()
        else:
            geometry = None
            balls = []
            enemy_robots = {}
            ally_robots = {}

        if request.field_border and geometry:
            obstacles.append(FieldBorderObstacle(geometry))

        if request.penalty_area and geometry:
            obstacles.append(PenaltyAreaObstacle(geometry, FieldSide.RIGHT))
            obstacles.append(PenaltyAreaObstacle(geometry, FieldSide.LEFT))

        if request.center_area:
            # Center Radius hardcoded
            obstacles.append(GenericCircleObstacle(Vector2D(0, 0), 500))

        if request.ball and balls:
            ball = balls[0]  # getting the first ball, maybe revise that...
            obstacles.append(
                GenericCircleObstacle(Vector2D(ball.position_x, ball.position_y), 60)
            )  # Ball radius 50mm + 10mm (safety)

        for enemy_id in request.enemy_ids:
            if enemy_id in enemy_robots:
                robot = enemy_robots[enemy_id]
                state = State(
                    Vector2D(robot.position_x, robot.position_y),
                    Vector2D(robot.velocity_x, robot.velocity_y),
                )
                obstacles.append(EnemyRobotObstacle(state, radius=190))

        for ally_id in request.ally_ids:
            if ally_id in ally_robots and ally_id != request.id:
                robot = ally_robots[ally_id]
                state = State(
                    Vector2D(robot.position_x, robot.position_y),
                    Vector2D(robot.velocity_x, robot.velocity_y),
                )
                obstacles.append(
                    AllyRobotObstacle(
                        state,
                        robot_data[ally_id]["trajectory"],
                        time_offset=robot_data[ally_id]["time_offset"],
                        radius=190,
                    )
                )

        return obstacles
