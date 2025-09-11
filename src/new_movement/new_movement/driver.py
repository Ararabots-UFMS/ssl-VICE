from new_movement.entities.Trajectory import Trajectory
from new_movement.planner import Planner
from new_movement.entities.States import Vector2D, State
from new_movement.entities.obstacles import Obstacle
from new_movement.utilities.obstacle_factory import ObstacleFactory

from system_interfaces.msg import ControlCommand, RobotControlCommand
from system_interfaces.srv import StrategyCommand, UpdateObstacle

from strategy.blackboard import Blackboard

import rclpy
from rclpy.node import Node


class PathDriver(Node):
    def __init__(self):
        super().__init__('path_driver')
        self.blackboard = Blackboard()
        # Publish Control
        self.publisher = self.create_publisher(ControlCommand, 'control_command', 10)
        self.control_timer = self.create_timer(0.01, self.publish_control)

        # Update Target Service
        self.planner = Planner(200)
        self.update_target_service = self.create_service(StrategyCommand, 'strategy_command', self.update_target)

        # Update Obstacles
        self.obstacle_factory = ObstacleFactory(self.blackboard)
        self.update_obstacles_service = self.create_service(UpdateObstacle, 'update_obstacles', self.update_obstacles)

        # Online Collision Check
        # self.collision_timer = self.create_timer(0.5, self.check_collision)

        # Robot data dictionary: id -> {trajectory, time_offset, obstacles}
        self.robot_data: dict[int, dict] = {}

        self.last_time = self.get_clock().now()

        self.driver_init()

        self.APPEND_POSITION_THRESHOLD = 300

    def driver_init(self):
        ally_robots_ids = self.blackboard.ally_robots.keys()

        for id in ally_robots_ids:
            # insert new found robot
            if id not in self.robot_data:
                cur_robot = self.blackboard.ally_robots[id]
                cur_state = State(Vector2D(cur_robot.position_x, cur_robot.position_y), Vector2D(cur_robot.velocity_x, cur_robot.velocity_y))

                init_trajectory = self.planner.find(cur_state, cur_state, [])
                
                self.robot_data[id] = {
                    'trajectory': init_trajectory,
                    'time_offset': 0.0,
                    'obstacles': [],
                    'last_obs_request': None
                }
            
            # Update obstacles for all ids
            if self.robot_data[id]['last_obs_request'] is not None:
                self.robot_data[id]['obstacles'] = self.obstacle_factory.create_obstacles(self.robot_data[id]['last_obs_request'], self.robot_data)

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
        for robot_id, robot_info in self.robot_data.items():

            robotControlCommand = RobotControlCommand()

            # self.get_logger().info(f"{robot_info['trajectory']}")
            # self.get_logger().info(f"{robot_info['time_offset']}")
            # self.get_logger().info(f"{robot_info['trajectory'].root}")
            # self.get_logger().info(f"{robot_info['trajectory'].get_state(robot_info['time_offset'])}")

            robotState = robot_info['trajectory'].get_state(robot_info['time_offset'])

            robotControlCommand.id = robot_id
            robotControlCommand.position_x = robotState.position.x / 1000 # to m/s
            robotControlCommand.position_y = robotState.position.y / 1000
            robotControlCommand.velocity_x = robotState.velocity.x / 1000
            robotControlCommand.velocity_y = robotState.velocity.y / 1000

            controlCommandList.append(robotControlCommand)

            if robot_info['time_offset'] <= robot_info['trajectory'].get_total_duration():
                robot_info['time_offset'] += dt
            else:
                robot_info['time_offset'] += robot_info['trajectory'].get_total_duration()

        controlCommand.command = controlCommandList

        self.publisher.publish(controlCommand)

    def replan(self, robot_id: int, new_destination: State) -> None:
        if robot_id not in self.robot_data:
            return
        
        cur_state = self.robot_data[robot_id]['trajectory'].get_state(self.robot_data[robot_id]['time_offset'])
        new_trajectory = self.planner.find(cur_state, new_destination, self.robot_data[robot_id]['obstacles'])

        if new_trajectory is None:
            #TODO if NOne, then it means no trajectory was found, so need to implement a fallback here
            return
        
        self.robot_data[robot_id]['trajectory'] = new_trajectory
        self.robot_data[robot_id]['time_offset'] = 0.0  # Reset Time offset
    
    def check_collision(self):
        pass

    def update_obstacles(self, request, response):
        self.driver_init()
        if request.id not in self.robot_data:
            response.success = False
            return response

        new_obstacles: list[Obstacle] = self.obstacle_factory.create_obstacles(request, self.robot_data)

        self.robot_data[request.id]['obstacles'] = new_obstacles
        self.robot_data[request.id]['last_obs_request'] = request

        response.success = True
        return response

    def update_target(self, request, response):
        self.driver_init()
        if request.id not in self.robot_data:
            response.success = False
            return response

        new_destination: State = State(Vector2D(request.position_x, request.position_y), Vector2D(request.velocity_x, request.velocity_y))

        self.replan(request.id, new_destination)

        response.success = True
        return response
    

def main(args=None):
    rclpy.init(args=args)
    node = PathDriver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()