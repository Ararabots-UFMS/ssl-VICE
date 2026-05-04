from movement_interfaces.msg import MovementCommandArray, TargetArray, Target
from movement_interfaces.srv import SetStaticObstacles, SetGoalKeeper
from system_interfaces.msg import VisionMessage
from system_interfaces.srv import GetGameConfig

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

class MovementManager(Node):
    def __init__(self):
        super().__init__('MovementManager')

        self.get_logger().info('MovementManager node has been started.')

        self.MovementCommandArray_sub = self.create_subscription(MovementCommandArray, 'MovementCommandTopic', self.movement_command_callback, 10)
        self.VisionMessage_sub = self.create_subscription(VisionMessage, 'VisionTopic', self.vision_message_callback, 10)

        self.StaticObstacles_srv = self.create_client(SetStaticObstacles, 'SetStaticObstacles')
        self.GoalKeeper_srv = self.create_client(SetGoalKeeper, 'SetGoalKeeper')
        self.TeamColor_srv = self.create_client(GetGameConfig, 'get_game_config')

        self._target_array_pub = self.create_publisher(TargetArray, 'movement_manager/targets', 10)

        self._movement_commands = None
        self.yellow_robots = None
        self.blue_robots = None

        self._team_color_yellow = None
        self._static_obstacles = None
        self._goal_keeper_id = None

        self.create_timer(0.1, self._poll_game_config)
        self.create_timer(0.1, self._poll_static_obstacles)
        self.create_timer(0.1, self._poll_goal_keeper)


    def movement_command_callback(self, msg):
        self._movement_commands = msg.commands
        self.try_publish_targets()

    def vision_message_callback(self, msg):
        self.yellow_robots = msg.yellow_robots
        self.blue_robots = msg.blue_robots
        self.try_publish_targets()

    def _call_service_sync(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if not future.done():
            return None
        if future.exception() is not None:
            return None
        return future.result()

    def _poll_game_config(self):
        if not self.TeamColor_srv.wait_for_service(timeout_sec=1.0):
            return
        resp = self._call_service_sync(self.TeamColor_srv, GetGameConfig.Request())
        self._team_color_yellow = resp.is_team_color_yellow

    def _poll_static_obstacles(self):
        if not self.StaticObstacles_srv.wait_for_service(timeout_sec=1.0):
            return
        resp = self._call_service_sync(self.StaticObstacles_srv, SetStaticObstacles.Request())
        self._static_obstacles = {
            'center_circle': resp.center_circle,
        }

    def _poll_goal_keeper(self):
        if not self.GoalKeeper_srv.wait_for_service(timeout_sec=1.0):
            return
        resp = self._call_service_sync(self.GoalKeeper_srv, SetGoalKeeper.Request())
        self._goal_keeper_id = resp.robot_id

    def try_publish_targets(self):
        if self._movement_commands is None:
            return
        if self._team_color_yellow is None:
            return
        if self._team_color_yellow:
            if self.yellow_robots is None:
                return
        else:
            if self.blue_robots is None:
                return
        if self._static_obstacles is None:
            return
        if self._goal_keeper_id is None:
            return

        target_array = self.Target_Array()
        self._target_array_pub.publish(target_array)

    def Target_Array(self):
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
    executor = MultiThreadedExecutor()
    executor.add_node(movement_manager)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        movement_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
