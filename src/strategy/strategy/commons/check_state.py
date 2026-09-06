from strategy.behaviour import LeafNode, TaskStatus
from system_interfaces.msg._game_state import GameState

class CheckState(LeafNode):
    def __init__(self, name, desired_states):
        super().__init__(name)
        self.desired_states = desired_states
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