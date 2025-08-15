from new_movement.entities.Trajectory import Trajectory
from new_movement.planner import Planner
from new_movement.entities.States import Vector2D, State
from new_movement.entities.obstacles import Obstacle
from system_interfaces.msg import ControlCommand, RobotControlCommand
from system_interfaces.srv import StrategyCommand

from strategy.blackboard import Blackboard

import rclpy
from rclpy.node import Node


class PathDriver(Node):
    def __init__(self):
        super().__init__('path_driver')
        # Publish Control
        self.publisher = self.create_publisher(ControlCommand, 'control_command', 10)
        self.control_timer = self.create_timer(0.1, self.publish_control)

        # Update Target Service
        self.planner = Planner(50)
        self.update_target_service = self.create_service(StrategyCommand, 'strategy_command', self.update_target)

        # Update Obstacles
        # self.update_obstacles_service = self.create_service(UpdateObstacles, 'update_obstacles', self.update_obstacles)

        # Online Collision Check
        # self.collision_timer = self.create_timer(0.5, self.check_collision)

        self.blackboard = Blackboard()

        self.cur_trajectories: list[Trajectory] = []
        self.cur_ids: list[int] = []
        self.time_offset: list[float] = []
        self.obstacles: list[list[Obstacle]] = []

        self.driver_init()

        self.APPEND_POSITION_THRESHOLD = 300

    def driver_init(self):
        ally_robots_ids = self.blackboard.ally_robots.keys()

        for id in ally_robots_ids:
            if not id in self.cur_ids:
                cur_robot = self.blackboard.ally_robots[id]
                cur_state = State(Vector2D(cur_robot.position_x, cur_robot.position_y), Vector2D(cur_robot.velocity_x), cur_robot.velocity_y)

                init_trajectory = self.planner.find(cur_state, cur_state, [])
                
                self.cur_ids.append(id)
                self.cur_trajectories.append(init_trajectory)
                self.time_offset.append(0.0)

    def publish_control(self):
        controlCommand = ControlCommand()
        controlCommandList = []
        for i, id in enumerate(self.cur_ids):
            robotControlCommand = RobotControlCommand()

            robotState = self.cur_trajectories[i].get_state(self.time_offset[i])

            robotControlCommand.id = id
            robotControlCommand.position_x = robotState.position.x
            robotControlCommand.position_y = robotState.position.y
            robotControlCommand.velocity_x = robotState.velocity.x
            robotControlCommand.velocity_y = robotState.velocity.y

            controlCommandList.append(robotControlCommand)

            if self.time_offset[i] < self.cur_trajectories[i].get_total_duration():
                self.time_offset[i] += 0.1

        controlCommand.command = controlCommandList

        self.publisher.publish(controlCommand)

    def replan(self, idx: int, new_destination: State) -> None:
        cur_state = self.cur_trajectories[idx].get_state(self.time_offset[idx]) #TODO Use real state
        new_trajectory = self.planner.find(cur_state, new_destination, []) #TODO Add the obstacles
        
        self.cur_trajectories[idx] = new_trajectory
        self.time_offset[idx] = 0.0 # Reset Time offset
    
    def check_collision(self):
        pass

    def update_obstacles(self, request, response):
        pass

    def update_target(self, request, response):
        if not request.id in self.cur_ids:
            self.driver_init()
            if not request.id in self.cur_ids:
                response.sucess = False
                return response

        idx = self.cur_ids.index(request.id)

        cur_destination: State = self.cur_trajectories[idx].get_destination()
        new_destination: State = State(Vector2D(request.position_x, request.position_y), Vector2D(request.velocity_x, request.velocity_y))

        destinations_distance = new_destination.position.distance(cur_destination.position)

        if destinations_distance < self.APPEND_POSITION_THRESHOLD:
            bridge_path: Trajectory = self.planner.find(cur_destination, new_destination, []) #TODO Add the obstacles
            self.cur_trajectories[idx].append(bridge_path)
        
            self.planner.optimizer.optimize(self.cur_trajectories[idx], self.planner.generator, self.planner.generator, [], initial_time = self.time_offset[idx]) #TODO add obstacles
        else:
            self.replan(idx, new_destination)

        response.sucess = True
        return response