from new_movement.entities.StaticObstacle import (
    FieldBorderObstacle,
    PenaltyAreaObstacle,
    GenericCircleObstacle,
    FieldSide,
)
from new_movement.entities.DynamicObstacles import AllyRobotObstacle, EnemyRobotObstacle
from new_movement.entities.States import State, Vector2D
from typing import Dict, List, Optional
from new_movement.entities.Trajectory import Trajectory


class ObstacleFactory:
    def __init__(self):
        pass

    def create_obstacles(
        self,
        robot_id: int,
        config,  # PlanningOptions or similar msg/obj with booleans
        geometry,
        balls,
        enemy_robots,
        ally_robots,
        ally_info: Optional[Dict[int, any]] = None, # Dict[int, TrajectoryPointMsg]
    ) -> list:
        obstacles = []
        ally_info = ally_info or {}

        # 1. Field Border (Always on if geometry exists)
        if geometry:
            try:
                obstacles.append(FieldBorderObstacle(geometry))
            except Exception:
                pass

        # 2. Toggleable Penalty Area
        planning_opts = getattr(config, 'planning_options', config)
        
        if getattr(planning_opts, 'avoid_penalty_area', True) and geometry:
            try:
                obstacles.append(PenaltyAreaObstacle(geometry, FieldSide.RIGHT))
                obstacles.append(PenaltyAreaObstacle(geometry, FieldSide.LEFT))
            except Exception:
                pass

        # 3. Toggleable Center Area
        if getattr(planning_opts, 'avoid_center_area', False):
            try:
                obstacles.append(GenericCircleObstacle(Vector2D(0, 0), 500))
            except Exception:
                pass

        # 4. Ball (Usually always on)
        if getattr(planning_opts, 'avoid_ball', True) and balls:
            try:
                ball = balls[0]
                obstacles.append(
                    GenericCircleObstacle(Vector2D(ball.position_x, ball.position_y), 60)
                )
            except Exception:
                pass

        # 5. Enemy Robots (Always on)
        for enemy in enemy_robots:
            try:
                state = State(
                    Vector2D(enemy.position_x, enemy.position_y),
                    Vector2D(enemy.velocity_x, enemy.velocity_y),
                )
                # radius = 90 (enemy robot) + 90 (own robot) + 20 (safety) = 200
                obstacles.append(EnemyRobotObstacle(state, radius=200))
            except Exception:
                pass

        # 6. Ally Robots (With Trajectory Support)
        for ally in ally_robots:
            a_id = ally.id
            if a_id == robot_id:
                continue  # Self-exclusion

            try:
                state = State(
                    Vector2D(ally.position_x, ally.position_y),
                    Vector2D(ally.velocity_x, ally.velocity_y),
                )
                
                if a_id in ally_info:
                    info = ally_info[a_id]
                    # Check if the overhead point contains a valid trajectory
                    if hasattr(info, 'trajectory') and len(info.trajectory.segments) > 0:
                        obstacles.append(
                            AllyRobotObstacle(
                                state,
                                Trajectory.from_msg(info.trajectory),
                                time_offset=info.timestamp,
                                radius=190
                            )
                        )
                    else:
                        # Fallback to simple prediction tube
                        obstacles.append(EnemyRobotObstacle(state, radius=200))
                else:
                    # Fallback if no tracker info available
                    obstacles.append(EnemyRobotObstacle(state, radius=200))
            except Exception:
                pass

        return obstacles
