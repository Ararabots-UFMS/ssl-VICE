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

        self._target_array_pub = self.create_publisher(TargetArray, 'TargetArrayTopic', 10)

        self.__game_config_inflight = False
        self._static_obstacles_inflight = False
        self._goal_keeper_inflight = False

        self._movement_commands = None
        self.yellow_robots = None
        self.blue_robots = None

        self._team_color_yellow = None
        self._static_obstacles = None
        self._goal_keeper_id = None

        self.create_timer(0.5, self._poll_game_config)
        self.create_timer(0.5, self._poll_static_obstacles)
        self.create_timer(0.5, self._poll_goal_keeper)


    def movement_command_callback(self, msg):
        self._movement_commands = msg.commands
        self.try_publish_targets()

    def vision_message_callback(self, msg):
        self.yellow_robots = msg.yellow_robots
        self.blue_robots = msg.blue_robots
        self.try_publish_targets()

    def _poll_game_config(self):
        if self.__game_config_inflight:
            return
        if not self.TeamColor_srv.service_is_ready():
            return
        self.__game_config_inflight = True
        future = self.TeamColor_srv.call_async(GetGameConfig.Request())
        def done(fut):
            self.__game_config_inflight = False
            try:
                resp = fut.result()
                self._team_color_yellow = bool(resp.is_team_color_yellow)
                self.get_logger().info(f"GameConfig updated: is_team_color_yellow={self._team_color_yellow}")
            except Exception as e:
                self.get_logger().error(f"Failed to call get_game_config service: {e}")
        future.add_done_callback(done)

    def _poll_static_obstacles(self):
        if self._static_obstacles_inflight:
            return
        if not self.StaticObstacles_srv.service_is_ready():
            return
        self._static_obstacles_inflight = True
        future = self.StaticObstacles_srv.call_async(SetStaticObstacles.Request())
        def done(fut):
            self._static_obstacles_inflight = False
            try:
                resp = fut.result()
                self._static_obstacles = {
                    'border_area': bool(resp.border_area),
                    'center_circle': bool(resp.center_circle)
                }
                self.get_logger().info(f"Static obstacles updated: {self._static_obstacles}")
            except Exception as e:
                self.get_logger().error(f"Failed to call SetStaticObstacles service: {e}")
        future.add_done_callback(done)

    def _poll_goal_keeper(self):
        if self._goal_keeper_inflight:
            return
        if not self.GoalKeeper_srv.service_is_ready():
            return
        self._goal_keeper_inflight = True
        future = self.GoalKeeper_srv.call_async(SetGoalKeeper.Request())
        def done(fut):
            self._goal_keeper_inflight = False
            try:
                resp = fut.result()
                self._goal_keeper_id = resp.robot_id
                self.get_logger().info(f"Goal keeper updated: robot_id={self._goal_keeper_id}")
            except Exception as e:
                self.get_logger().error(f"Failed to call SetGoalKeeper service: {e}")
        future.add_done_callback(done)

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
        goalkeeper_target = None

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
            target.planning_options.avoid_penalty_area = self._static_obstacles['border_area']
            target.planning_options.avoid_center_circle = self._static_obstacles['center_circle']
            target.target_pos = cmd.target_pos

            if robot.id == self._goal_keeper_id:
                goalkeeper_target = target
            else:
                normal_targets.append(target)

        if goalkeeper_target is not None:
            normal_targets.append(goalkeeper_target)

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
