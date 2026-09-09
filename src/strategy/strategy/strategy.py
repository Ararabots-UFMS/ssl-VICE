import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy

from strategy.context import GameConfig, TickContext, TreeDeps
from strategy.root import RootTree
from typing import Iterable, Optional
from std_msgs.msg import Bool
from system_interfaces.msg import GameState
from system_interfaces.srv import GetGameConfig
from strategy.skills.skills import Skill
from strategy.movement_handler import MovementHandler


class Strategy(Node):
    """Only ROS node in the strategy package.

    It owns every subscription, every service client and the tick. The behaviour
    tree below it receives an immutable snapshot of the world once per tick and returns skills
    """

    def __init__(self):
        super().__init__("strategy_node")
        self.get_logger().info("Strategy node initialized")

        # The GUI can hand movement over to its manual debug tool. Both publish to
        # movement_manager/commands and the manager keeps only the last array, so this
        # node has to actually go quiet rather than merely be ignored.
        #
        # Latched by the GUI, so the current mode arrives even if this node starts later.
        # Defaults to enabled: with no GUI running, strategy is the only driver.
        self._enabled = True
        self._enabled_sub = self.create_subscription(
            Bool,
            "strategy/enabled",
            self._enabled_callback,
            QoSProfile(
                depth=1,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
            ),
        )

        self._latest_game_state: Optional[GameState] = None
        self._game_config: Optional[GameConfig] = None
        self.deps = TreeDeps(logger=self.get_logger())
        self.root = RootTree("RootStrategy", self.deps)

        self.movement_handler = MovementHandler(self)

        self.create_subscription(GameState, "game_state", self._on_game_state, 10)

        self._game_config_client = self.create_client(GetGameConfig, "get_game_config")
        self._game_config_poll_timer = self.create_timer(5, self._poll_game_config)

        self.timer = self.create_timer(0.1, self.run)

    def _enabled_callback(self, msg: Bool) -> None:
        if bool(msg.data) == self._enabled:
            return
        self._enabled = bool(msg.data)
        self.get_logger().info(
            f"Strategy {'enabled' if self._enabled else 'disabled (GUI manual control)'}"
        )

    def _on_game_state(self, msg: GameState) -> None:
        """Hand the newest frame over and touch nothing else.

        Rebinding one attribute is atomic, and rclpy hands us a freshly deserialized
        message every time, so the object the tick is holding is never mutated
        underneath it.
        """
        self._latest_game_state = msg

    def _poll_game_config(self) -> None:
        if self._game_config is not None:
            return
        if not self._game_config_client.service_is_ready():
            return

        future = self._game_config_client.call_async(GetGameConfig.Request())

        def done(fut):
            try:
                self._game_config = GameConfig.from_response(fut.result())
            except Exception as e:
                self.get_logger().error(f"Failed to fetch game config: {e}")

        future.add_done_callback(done)

    def run(self) -> None:
        # Returns before the tree runs, not just before publishing: the movement
        # handler's kick and orientation calls would otherwise fight the debug tool.
        if not self._enabled:
            return

        # One read of each shared name per tick. The whole BT works off these
        # locals, so nothing that arrives mid-tick can change what they see.
        msg = self._latest_game_state
        config = self._game_config

        if msg is None:
            return

        context = TickContext.from_game_state(msg, config)

        status, action = self.root.run(context)

        if action is None:
            return

        if isinstance(action, Iterable) and not isinstance(action, Skill):
            skill_list = [s for s in action if hasattr(s, "robot_id")]
        else:
            skill_list = [action] if hasattr(action, "robot_id") else []

        latest: dict[int, Skill] = {skill.robot_id: skill for skill in skill_list}

        self.movement_handler.handle_movement(latest.values())

def main(args=None):
    rclpy.init(args=args)
    strategy_node = Strategy()
    rclpy.spin(strategy_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
