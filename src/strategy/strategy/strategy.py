import os
import time
from typing import Iterable, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from std_msgs.msg import Bool
from system_interfaces.msg import GameState
from system_interfaces.srv import GetGameConfig

from strategy import diagnostico
from strategy.context import GameConfig, TickContext, TreeDeps
from strategy.movement_handler import MovementHandler
from strategy.root import RootTree
from strategy.skills.skills import Skill

# A GetGameConfig call that has not answered in this long is retried: a call made
# before the game watcher was up hung forever and kept the tree in RUNNING.
CONFIG_RETRY_S = 3.0
# A/B movement test: robot 3 shuttles between two fixed points. The flag file holds
# the shuttle length in mm. Asked by Felipe: is slowness movement or strategy?
ARQUIVO_AB = "/tmp/ararabots_ab"
ROBO_AB = 3


class Strategy(Node):
    """Only ROS node in the strategy package.

    It owns every subscription, every service client and the tick. The behaviour
    tree below it receives an immutable snapshot of the world once per tick and returns skills
    """

    def __init__(self, wait_for_service: bool = True):
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
        self._config_future = None
        self._config_requested_at = 0.0
        self.deps = TreeDeps(logger=self.get_logger())
        self.root = RootTree("RootStrategy", self.deps)

        self.movement_handler = MovementHandler(self, wait_for_services=wait_for_service)

        self.create_subscription(GameState, "game_state", self._on_game_state, 10)

        self._game_config_client = self.create_client(GetGameConfig, "get_game_config")
        self._game_config_poll_timer = self.create_timer(0.5, self._poll_game_config)

        self._t0_kickoff: Optional[float] = None
        self._kickoff_tick = 0
        self._ab_target: Optional[tuple] = None
        self._published_once = False

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
            self._game_config_poll_timer.cancel()
            return
        if self._config_future is not None:
            if time.monotonic() - self._config_requested_at < CONFIG_RETRY_S:
                return
            self._config_future = None
        if not self._game_config_client.service_is_ready():
            return

        self._config_requested_at = time.monotonic()
        future = self._game_config_client.call_async(GetGameConfig.Request())
        self._config_future = future

        def done(fut):
            try:
                self._game_config = GameConfig.from_response(fut.result())
            except Exception as e:
                self.get_logger().error(f"Failed to fetch game config: {e}")
            self._config_future = None

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
        # Sign of life the measurement procedure greps for (docs/auditoria §7).
        diagnostico.log("[JG] CheckState cmd=%r" % (context.referee_command,))

        status, action = self.root.run(context)

        if action is None:
            skills: List[Skill] = []
        elif isinstance(action, Iterable) and not isinstance(action, Skill):
            skills = [s for s in action if hasattr(s, "robot_id")]
        else:
            skills = [action] if hasattr(action, "robot_id") else []

        if diagnostico.ativo():
            self._diagnose_kickoff(context, status, skills)

        if not skills:
            return

        latest: dict[int, Skill] = {skill.robot_id: skill for skill in skills}

        if os.path.exists(ARQUIVO_AB):
            self._ab_test(context, latest)

        self.movement_handler.handle_movement(latest.values())

        if diagnostico.ativo() and not self._published_once:
            self._published_once = True
            diagnostico.log("[LG] PRIMEIRA PUBLICACAO t=%.3f robos=%s"
                            % (time.monotonic() - (self._t0_kickoff or time.monotonic()),
                               [s.robot_id for s in latest.values() if s.target_x is not None]))

    def _diagnose_kickoff(self, context: TickContext, status, skills: List[Skill]) -> None:
        """[LG] the first 25 ticks after the start: slow to command, or slow to move?

        Our robots started moving 0.68-1.03 s after the start, the opponent at 0.08 s.
        Time zero is the referee's start, not the node's.
        """
        if context.referee_command not in ("FORCE_START", "NORMAL_START"):
            return
        if self._t0_kickoff is None:
            self._t0_kickoff = time.monotonic()
        self._kickoff_tick += 1
        if self._kickoff_tick > 25:
            return
        targets = " ".join("r%d:(%.0f,%.0f)" % (s.robot_id, s.target_x or 0.0, s.target_y or 0.0)
                           for s in skills)
        diagnostico.log("[LG] ciclo=%d t=%.3f status=%s comandos=%d %s"
                        % (self._kickoff_tick, time.monotonic() - self._t0_kickoff,
                           getattr(status, "name", status), len(skills), targets))

    def _ab_test(self, context: TickContext, latest: dict) -> None:
        skill = latest.get(ROBO_AB)
        robot = context.ally_robots.get(ROBO_AB)
        if skill is None or skill.target_x is None or skill.target_y is None:
            return
        try:
            with open(ARQUIVO_AB) as f:
                length = float(f.read().strip() or 6000.0)
        except (OSError, ValueError):
            length = 6000.0
        a, b = (-length / 2.0, 2000.0), (length / 2.0, 2000.0)
        target = self._ab_target or b
        if robot is not None:
            d = ((robot.position_x - target[0]) ** 2 + (robot.position_y - target[1]) ** 2) ** 0.5
            if d < 300.0:
                target = a if target == b else b
        self._ab_target = target
        skill.target_x, skill.target_y = target
        diagnostico.log("[AB] r%d alvo fixo=(%.0f,%.0f)" % (ROBO_AB, target[0], target[1]))


def main(args=None):
    rclpy.init(args=args)
    strategy_node = Strategy(wait_for_service=True)
    try:
        rclpy.spin(strategy_node)
    except KeyboardInterrupt:
        pass
    finally:
        strategy_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
