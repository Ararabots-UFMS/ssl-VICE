from movement_interfaces.msg import MovementCommandArray, TargetArray, Target
from movement_interfaces.srv import SetStaticObstacles, SetGoalKeeper
from system_interfaces.msg import VisionMessage
from system_interfaces.srv import GetGameConfig

import rclpy
from rclpy.node import Node

class MovementManager(Node):
    def __init__(self):
        super().__init__('MovementManager')

        self.get_logger().info('MovementManager node has been started.')

        self._movement_command_sub = self.create_subscription(MovementCommandArray, 'MovementCommandTopic', self.movement_command_callback, 10)
        self._vision_message_sub = self.create_subscription(VisionMessage, 'VisionTopic', self.vision_message_callback, 10)

        self._static_obstacles_srv= self.create_service(SetStaticObstacles, 'SetStaticObstacles', self._set_static_obstacles)
        self._goal_keeper_srv = self.create_service(SetGoalKeeper, 'SetGoalKeeper', self._set_goal_keeper)
        self._team_color_cli = self.create_client(GetGameConfig, 'get_game_config')

        self._target_array_pub = self.create_publisher(TargetArray, 'movement_manager/targets', 10)

        self._movement_commands = None
        self.yellow_robots = None
        self.blue_robots = None

        self._team_color_yellow = None
        self._static_obstacles = None
        self._goal_keeper_id = None

        self.create_timer(0.1, self._poll_game_config)


    def movement_command_callback(self, msg):
        self._movement_commands = msg.commands
        self.try_publish_targets()

    def vision_message_callback(self, msg):
        self.yellow_robots = msg.yellow_robots
        self.blue_robots = msg.blue_robots
        self.try_publish_targets()

    def _poll_game_config(self):
        if not self._team_color_cli.wait_for_service(timeout_sec=1.0):
            return
        future = self._team_color_cli.call_async(GetGameConfig.Request())
        future.add_done_callback(self._on_game_config_response)

    def _on_game_config_response(self, future):
        try:
            resp = future.result()
            self._team_color_yellow = bool(resp.is_team_color_yellow)
        except Exception as e:
            self.get_logger().error(f'Failed to call get_game_config: {e}')

    def _set_static_obstacles(self, request, response):
        self._static_obstacles = {
            'border_area':bool(request.border_area),
            'center_circle':bool(request.center_circle)
        }
        return response

    def _set_goal_keeper(self, request, response):
        self._goal_keeper_id = int(request.robot_id)
        return response

    def _robots_ready(self):
        if self._team_color_yellow is None:
            return False
        robots = self.yellow_robots if self._team_color_yellow else self.blue_robots
        return robots is not None

    def _ready_to_publish(self):
        return all([
            self._movement_commands is not None,
            self._static_obstacles is not None,
            self._goal_keeper_id is not None,
            self._robots_ready(),
        ])

    def try_publish_targets(self):
        if not self._ready_to_publish():
            return
        self._target_array_pub.publish(self._build_target_array())

    def _build_target_array(self):
        if self._team_color_yellow:
            robots = self.yellow_robots
        else:
            robots = self.blue_robots

        cmd_by_id = {cmd.robot_id: cmd for cmd in self._movement_commands}

        normal_targets = []

        msg = TargetArray()

        for robot in robots:
            cmd = cmd_by_id.get(robot.id)
            if cmd is None:
                continue

            target = Target()
            target.robot_id = robot.id
            target.initial_pos.x = robot.position_x
            target.initial_pos.y = robot.position_y
            target.initial_vel.x = robot.velocity_x
            target.initial_vel.y = robot.velocity_y
            target.planning_options.avoid_center_area = self._static_obstacles['center_circle']
            target.target_pos = cmd.target_pos

            if robot.id == self._goal_keeper_id:
                target.planning_options.avoid_penalty_area = False
            else:
                target.planning_options.avoid_penalty_area = True

            normal_targets.append(target)

        msg.targets = normal_targets

        return msg

def main(args=None):
    rclpy.init(args=args)
    movement_manager = MovementManager()
    try:
        rclpy.spin(movement_manager)
    except KeyboardInterrupt:
        pass
    finally:
        movement_manager.destroy_node()
        rclpy.shutdown()
