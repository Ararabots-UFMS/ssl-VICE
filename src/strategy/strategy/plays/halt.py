from system_interfaces.msg import GameState
from system_interfaces.srv import GetGameConfig
from strategy.strategy.behaviour import Selector, Sequence, LeafNode, TaskStatus
from strategy.strategy.tatics.stop_for_halt import halt

class checkState(LeafNode):
    def __init__(self, name, node, _desired_states, _referee_command):
        super.__init__(name, [])
        self.node = node
        self.referee_command = None
        self.desired_states = _desired_states
        self.referee_command = _referee_command

        def run(self):
            return(TaskStatus.SUCCESS, None) if self._referee_command in self._desired_states else (TaskStatus.FAILURE, None)
        
class HaltAction(LeafNode):
    def __init__(self, name, ally_robots = None):
        super().__init__(name, [])
        self.ally_robots = ally_robots

    def run(self):
        if not self.ally_robots:
            return TaskStatus.RUNNING, None
        
        executor = halt(ally_robots)
        return (TaskStatus.SUCCESS, None)
    
class Halt(Sequence):
    def __init__(self, name):
        super().__init__(name, [])

        self.game_state_sub = self.create_subscription(GameState, "game_state", self.game_state_callback, 10)
        
        commands = ['STOP'] 

        check_halt = checkState("CheckHalt", commands, self.referee_command)

        action = HaltAction("HaltAction")

        self.add_children([check_halt,])
    def game_state_callback(self, msg):
        self.referee_command = msg.referee.command