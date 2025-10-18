from system_interfaces.msg import GameState
from strategy.behaviour import Sequence, LeafNode, TaskStatus


class checkState(LeafNode):
    def __init__(self, name, _desired_states):
        super().__init__(name)
        self.desired_states = _desired_states
        self.referee_command = None
        self.create_subscription(GameState, "game_state", self.game_state_callback, 10)

    def game_state_callback(self, msg: GameState):
        self.referee_command = msg.referee.command

    def run(self):
        return (
            (TaskStatus.SUCCESS, None)
            if self.referee_command in self.desired_states
            else (TaskStatus.FAILURE, None)
        )


class HaltActionNode(LeafNode):
    def __init__(self, name):
        super().__init__(name)

    def run(self):
        return TaskStatus.SUCCESS, None


class Halt(Sequence):
    def __init__(self, name):
        super().__init__(name, [])

        self.referee_command = "HALT"

        commands = ["TIMEOUT_YELLOW", "TIMEOUT_BLUE", "HALT"]
        check_halt = checkState("CheckHalt", commands)
        action = HaltActionNode("HaltAction")
        self.add_children([check_halt, action])
