from typing import Dict, Optional

from movement.entities.obstacle.field_border_obstacle import FieldBorderObstacle
from movement.entities.obstacle.penalty_area_obstacle import PenaltyAreaObstacle
from movement.entities.obstacle.generic_circle_obstacle import GenericCircleObstacle
from movement.entities.obstacle.ally_robot_obstacle import AllyRobotObstacle
from movement.entities.obstacle.enemy_robot_obstacle import EnemyRobotObstacle
from movement.entities.motion.motion_state import MotionState
from movement.entities.trajectory.trajectory import Trajectory

from utils.math_util import Vector2D
from utils.field_util import FieldSide

# 90 (enemy robot) + 90 (own robot) + 20 (safety).
ENEMY_KEEP_OUT = 200.0
# The margin left for an opponent on a ball this robot is going for. At 200 two robots
# head-on at the same ball sit 204mm apart, one noisy frame from being pushed off it.
ENEMY_KEEP_OUT_AT_BALL = 185.0
# How near the ball an opponent counts as on it: the keep-out plus a robot at the ball.
CONTEST_RANGE = 300.0
# Room a route keeps from the ball's disc. Without it routes graze the edge, and the
# tracking error measured going round the ball (p90 70mm) puts the robot inside it.
BALL_ROUTE_CLEARANCE = 70.0


class ObstacleFactory:
    def __init__(self, logger=None):
        self.logger = logger

    def _warn(self, message: str) -> None:
        if self.logger is not None:
            self.logger.warn(message)

    @staticmethod
    def _as_robot_list(robots) -> list:
        """GameState's Robots[] sequences, tolerating the empty case."""
        return list(robots or [])

    @staticmethod
    def _enemy_keep_out(enemy: MotionState, contested_ball) -> float:
        if contested_ball is None:
            return ENEMY_KEEP_OUT
        ball = Vector2D(contested_ball.position_x, contested_ball.position_y)
        if enemy.position.distance(ball) < CONTEST_RANGE:
            return ENEMY_KEEP_OUT_AT_BALL
        return ENEMY_KEEP_OUT

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
        # Safety critical: a failure here propagates so the caller drops the plan
        # instead of driving a robot that believes the field is unbounded.
        if geometry:
            obstacles.append(FieldBorderObstacle(geometry))

        # 2. Toggleable Penalty Area
        planning_opts = getattr(config, 'planning_options', config)

        # Safety critical as well: entering the penalty area is a foul, so a broken
        # obstacle must abort planning rather than silently allow the shortcut.
        if getattr(planning_opts, 'avoid_penalty_area', True) and geometry:
            obstacles.append(PenaltyAreaObstacle(geometry, FieldSide.RIGHT))
            obstacles.append(PenaltyAreaObstacle(geometry, FieldSide.LEFT))

        # 3. Toggleable Center Area
        if getattr(planning_opts, 'avoid_center_area', False):
            try:
                obstacles.append(GenericCircleObstacle(Vector2D(0, 0), 500))
            except Exception as e:
                self._warn(f"Could not build the center area obstacle: {e}")

        # 4. Ball (Usually always on)
        avoid_ball = getattr(planning_opts, 'avoid_ball', True)
        if avoid_ball and balls:
            try:
                ball = balls[0]
                obstacles.append(
                    GenericCircleObstacle(
                        Vector2D(ball.position_x, ball.position_y),
                        60,
                        clearance=BALL_ROUTE_CLEARANCE,
                    )
                )
            except Exception as e:
                self._warn(f"Could not build the ball obstacle: {e}")

        # 5. Enemy Robots (Always on)
        # A robot that is not avoiding the ball is going for it.
        contested_ball = balls[0] if balls and not avoid_ball else None
        for enemy in self._as_robot_list(enemy_robots):
            try:
                state = MotionState(
                    Vector2D(enemy.position_x, enemy.position_y),
                    Vector2D(enemy.velocity_x, enemy.velocity_y),
                )
                obstacles.append(
                    EnemyRobotObstacle(
                        state, radius=self._enemy_keep_out(state, contested_ball)
                    )
                )
            except Exception as e:
                self._warn(f"Could not build the obstacle for enemy robot: {e}")

        # 6. Ally Robots (With Trajectory Support)
        for ally_data in self._as_robot_list(ally_robots):
            ally_id = ally_data.id
            if ally_id == robot_id:
                continue  # Self-exclusion

            try:
                state = MotionState(
                    Vector2D(ally_data.position_x, ally_data.position_y),
                    Vector2D(ally_data.velocity_x, ally_data.velocity_y),
                )
                
                if ally_id in ally_info:
                    info = ally_info[ally_id]
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
            except Exception as e:
                self._warn(f"Could not build the obstacle for ally robot {ally_id}: {e}")

        return obstacles
