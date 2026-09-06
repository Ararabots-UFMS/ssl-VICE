from system_interfaces.msg import GameState
from strategy.behaviour import Sequence, LeafNode, TaskStatus
from strategy.tatics.halt import HaltAction
from strategy.commons.check_state import CheckState


class HaltActionNode(LeafNode):
    def __init__(self, name):
        super().__init__(name)

        self.ally_robots = {}
        self.create_subscription(GameState, "game_state", self.game_state_callback, 10)

    def game_state_callback(self, msg: GameState):
        self.ally_robots = {r.id: r for r in msg.ally_robots}

    def run(self):
        if not self.ally_robots:
            return TaskStatus.RUNNING, None

        executor = HaltAction(self.ally_robots)

        return TaskStatus.SUCCESS, executor.execute()


class Halt(Sequence):
    def __init__(self, name):
        super().__init__(name, [])

        self.referee_command = "HALT"

        commands = ["HALT"]
        check_halt = CheckState("CheckHalt", commands)
        action = HaltActionNode("HaltAction")
        self.add_children([check_halt, action])
