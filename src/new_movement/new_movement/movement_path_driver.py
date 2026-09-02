from time import sleep

import rclpy
from rclpy.node import Node

from new_movement.entities.motion.motion_state import MotionState
from new_movement.entities.obstacle.obstacle import Obstacle
from new_movement.entities.obstacle.static_obstacle import StaticObstacle
from new_movement.local_planner.obstacle_factory import ObstacleFactory
from new_movement.local_planner.orchestrator import Orchestrator

from system_interfaces.msg import (
    ControlCommand,
    RobotControlCommand,
    GameState,
    TrajectoryMessage,
    RobotTrajectory,
    TrajectoryPoint,
)
from system_interfaces.srv import StrategyCommand, UpdateObstacle

from utils.math_util import Vector2D


class MovementPathDriver(Node):
    def __init__(self):
        super().__init__("movement_path_driver")

        self.ally_robots = {}
        self.enemy_robots = {}
        self.balls = []
        self.geometry = None

        # Publish Control
        self.publisher = self.create_publisher(ControlCommand, "control_command", 10)
        self.control_timer = self.create_timer(0.01, self.publish_control)

        # Publish Trajectories
        self.trajectory_publisher = self.create_publisher(
            TrajectoryMessage, "trajectory_topic", 10
        )
        self.trajectory_timer = self.create_timer(0.1, self.publish_trajectories_safe)

        # Update Target Service
        self.planner = Orchestrator()
        self.update_target_service = self.create_service(
            StrategyCommand, "strategy_command", self.update_target
        )

        # Update Obstacles
        self.obstacle_factory = ObstacleFactory()
        self.update_obstacles_service = self.create_service(
            UpdateObstacle, "update_obstacles", self.update_obstacles
        )

        # Obstacles Update Timer
        self.update_obstacles_timer = self.create_timer(
            0.5, self.update_obstacles_timer_callback
        )

        # Online Collision Check
        self.collision_timer = self.create_timer(0.5, self.check_collision)
        self.collision_look_ahead = 1.5  # Seconds

        # Robot data dictionary: id -> {trajectory, time_offset, obstacles}
        self.robot_data: dict[int, dict] = {}
        self.last_command = {}

        self.last_time = self.get_clock().now()

        # Subscription for aggregated game state
        self.game_state_sub = self.create_subscription(
            GameState, "game_state", self.game_state_callback, 10
        )
        
        sleep(1) # Gives time to game watcher update 

        self.driver_init()

    def game_state_callback(self, msg: GameState):
        self.ally_robots = {r.id: r for r in msg.ally_robots}
        self.enemy_robots = {r.id: r for r in msg.enemy_robots}
        self.balls = list(msg.balls)
        self.geometry = msg.geometry

    def driver_init(self):
        ally_robots = self.ally_robots
        ally_robots_ids = ally_robots.keys()

        for id in ally_robots_ids:
            # insert new found robot
            if id not in self.robot_data:
                try:
                    cur_robot = ally_robots[id]
                    cur_state = MotionState(
                        Vector2D(cur_robot.position_x, cur_robot.position_y),
                        Vector2D(cur_robot.velocity_x, cur_robot.velocity_y),
                    )

                    # Check if there is a last command for this robot
                    if id in self.last_command:
                        last_cmd = self.last_command[id]
                        target_state = MotionState(
                            Vector2D(last_cmd.position_x, last_cmd.position_y),
                            Vector2D(last_cmd.velocity_x, last_cmd.velocity_y),
                        )
                        init_trajectory = self.planner.find(cur_state, target_state, [])
                    else:
                        init_trajectory = self.planner.find(cur_state, cur_state, [])

                    self.robot_data[id] = {
                        "trajectory": init_trajectory,
                        "time_offset": 0.0,
                        "obstacles": [],
                        "last_obs_request": None,
                    }
                except Exception as e:
                    self.get_logger().warn(f"Erro ao inicializar robô {id}: {e}")
                    self.robot_data[id] = {
                        "trajectory": None,
                        "time_offset": 0.0,
                        "obstacles": [],
                        "last_obs_request": None,
                    }

            # Update obstacles for all ids
            if self.robot_data[id]["last_obs_request"] is not None:
                self.robot_data[id]["obstacles"] = (
                    self.obstacle_factory.create_obstacles(
                        self.robot_data[id]["last_obs_request"],
                        self.robot_data,
                        self.geometry,
                        self.balls,
                        self.enemy_robots,
                        self.ally_robots,
                    )
                )

        # Removing not longer found robot
        for traked_id in list(self.robot_data.keys()):
            if traked_id not in ally_robots_ids:
                self.robot_data.pop(traked_id)

    def publish_control(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds
        self.last_time = current_time

        controlCommand = ControlCommand()
        controlCommandList = []

        self.driver_init()
        try:
            for robot_id, robot_info in self.robot_data.items():
                trajectory = robot_info.get("trajectory")
                if trajectory is None:
                    continue
 
                robotControlCommand = RobotControlCommand()

                robotState = trajectory.get_state(robot_info["time_offset"])

                if robotState is None:
                    continue

                robotControlCommand.id = int(robot_id)
                robotControlCommand.position_x = float(robotState.position.x / 1000)
                robotControlCommand.position_y = float(robotState.position.y / 1000)
                robotControlCommand.velocity_x = float(robotState.velocity.x / 1000)
                robotControlCommand.velocity_y = float(robotState.velocity.y / 1000)

                controlCommandList.append(robotControlCommand)

                total_duration = trajectory.get_total_duration()
                if (
                    total_duration is not None
                    and robot_info["time_offset"] <= total_duration
                ):
                    robot_info["time_offset"] += dt
                else:
                    robot_info["time_offset"] = (
                        total_duration if total_duration is not None else 0.0
                    )
        except Exception as e:
            self.get_logger().error(f"Error in publish_control: {e}")

        controlCommand.command = controlCommandList

        self.publisher.publish(controlCommand)

    def publish_trajectories_safe(self):
        """error Treatment for publish_trajectories """
        try:
            self.publish_trajectories()
        except Exception as e:
            self.get_logger().warn(f"Erro ao publicar trajetórias: {e}")

    def publish_trajectories(self):
        """Publish the current trajectories for all robots"""
        trajectory_message = TrajectoryMessage()
        trajectory_list = []

        self.driver_init()

        for robot_id, robot_info in self.robot_data.items():
            trajectory = robot_info.get("trajectory")
            if trajectory is None:
                continue

            try:
                total_duration = trajectory.get_total_duration()
                if total_duration is None or total_duration <= 0:
                    continue

                robot_trajectory = RobotTrajectory()
                robot_trajectory.robot_id = int(robot_id)
                robot_trajectory.total_duration = float(total_duration)

                time_step = 0.1
                max_preview_time = 3.0
                current_time = robot_info["time_offset"]

                trajectory_points = []

                t = 0.0
                max_time = min(
                    max_preview_time, robot_trajectory.total_duration - current_time
                )
                while t <= max_time and max_time > 0:
                    future_time = current_time + t
                    if future_time <= robot_trajectory.total_duration:
                        state = trajectory.get_state(future_time)
                        if state is not None:
                            point = TrajectoryPoint()
                            point.x = float(state.position.x / 1000.0)
                            point.y = float(state.position.y / 1000.0)
                            point.velocity_x = float(state.velocity.x / 1000.0)
                            point.velocity_y = float(state.velocity.y / 1000.0)
                            point.timestamp = float(t)
                            trajectory_points.append(point)
                    t += time_step

                robot_trajectory.points = trajectory_points
                trajectory_list.append(robot_trajectory)

            except Exception as e:
                self.get_logger().warn(
                    f"Erro ao processar trajetória do robô {robot_id}: {e}"
                )
                continue

        trajectory_message.trajectories = trajectory_list
        self.trajectory_publisher.publish(trajectory_message)

    def replan(self, robot_id: int, new_destination: MotionState) -> bool:
        if robot_id not in self.robot_data:
            return False

        cur_state = self.robot_data[robot_id]["trajectory"].get_state(
            self.robot_data[robot_id]["time_offset"]
        )
        new_trajectory = self.planner.find(
            cur_state, new_destination, self.robot_data[robot_id]["obstacles"]
        )

        if new_trajectory.root is None or new_trajectory.get_state(0.0) is None:
            # TODO if None, then it means no trajectory was found, so need to implement a fallback here
            return False

        self.robot_data[robot_id]["trajectory"] = new_trajectory
        self.robot_data[robot_id]["time_offset"] = 0.0  # Reset Time offset
        self.last_time = self.get_clock().now()

        return True

    def check_collision(self):
        for id, robot in self.robot_data.items():
            total_time: float = 0.0
            time_step = 0.01
            while total_time <= self.collision_look_ahead:
                need_replan: bool = False
                for obs in robot["obstacles"]:
                    curPosition: Vector2D = (
                        robot["trajectory"].get_state(total_time + robot["time_offset"])
                    ).position
                    if isinstance(obs, StaticObstacle):
                        if obs.isCollidingAt(curPosition):
                            need_replan = True
                            break
                    else:
                        if obs.isCollidingAt(curPosition, total_time):
                            need_replan = True
                            break

                if need_replan is True:
                    self.replan(id, robot["trajectory"].get_destination())
                    self.get_logger().debug(f"Replan Activated for {id}")
                    break

                total_time += time_step

    def update_obstacles(self, request, response):
        self.driver_init()
        if request.id not in self.robot_data:
            self.get_logger().debug(
                f"Update obstacles ignored: robot {request.id} not found"
            )
            response.success = False
            return response

        new_obstacles: list[Obstacle] = self.obstacle_factory.create_obstacles(
            request,
            self.robot_data,
            self.geometry,
            self.balls,
            self.enemy_robots,
            self.ally_robots,
        )

        self.robot_data[request.id]["obstacles"] = new_obstacles
        self.robot_data[request.id]["last_obs_request"] = request

        response.success = True
        return response

    def update_obstacles_timer_callback(self):
        for robot_id, robot_info in self.robot_data.items():
            if robot_info["last_obs_request"] is not None:
                new_obstacles = self.obstacle_factory.create_obstacles(
                    robot_info["last_obs_request"],
                    self.robot_data,
                    self.geometry,
                    self.balls,
                    self.enemy_robots,
                    self.ally_robots,
                )
                self.robot_data[robot_id]["obstacles"] = new_obstacles

    def update_target(self, request, response):
        self.driver_init()
        if request.id not in self.robot_data:
            response.success = False
            return response

        self.last_command[request.id] = request

        new_destination: MotionState = MotionState(
            Vector2D(request.position_x, request.position_y),
            Vector2D(request.velocity_x, request.velocity_y),
        )

        status: bool = self.replan(request.id, new_destination)

        response.success = status
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MovementPathDriver()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
