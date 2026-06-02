from movement_interfaces.msg import MovementCommandArray, TargetArray, Target
from movement_interfaces.srv import SetStaticObstacles, SetGoalKeeper
from system_interfaces.msg import GameState

import rclpy
from rclpy.node import Node

class MovementManager(Node):
    def __init__(self):
        super().__init__('MovementManager')

        self.get_logger().info('MovementManager node has been started.')

        self._movement_command_sub = self.create_subscription(MovementCommandArray, 'MovementCommandTopic', self.movement_command_callback, 10)
        self._game_state_sub = self.create_subscription(GameState, 'game_state', self.game_state_callback, 10)
        self._static_obstacles_srv= self.create_service(SetStaticObstacles, 'SetStaticObstacles', self._set_static_obstacles)
        self._goal_keeper_srv = self.create_service(SetGoalKeeper, 'SetGoalKeeper', self._set_goal_keeper)

        self._target_array_pub = self.create_publisher(TargetArray, 'movement_manager/targets', 10)

        self._movement_commands = None

        self._robots = None
        self._static_obstacles = None
        self._goal_keeper_id = None

    def movement_command_callback(self, msg):
        self._movement_commands = msg.commands
        self.try_publish_targets()

    def game_state_callback(self, msg):
        self._robots = msg.ally_robots
        self.try_publish_targets()

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
        return self._robots is not None

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
        robots = self._robots

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
