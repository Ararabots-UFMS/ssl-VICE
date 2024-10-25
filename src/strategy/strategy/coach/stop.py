from strategy.behaviour import LeafNode, Sequence, TaskStatus
from strategy.blackboard import Blackboard


class CheckState(LeafNode):
    def __init__(self, name, _desired_states):
        self.name = name
        self.blackboard = Blackboard()
        self.desired_states = _desired_states

    def run(self):
        if self.blackboard.referee.command in self.desired_states:
            return TaskStatus.SUCCESS, "None"

        return TaskStatus.FAILURE, "None"
    
class StopAction(LeafNode):
    def __init__(self, name):
        super().__init__(name)

    def run(self):
        return TaskStatus.SUCCESS, "EVERYONE STOP"
    
class Stop(Sequence):
    def __init__(self, name):
        super().__init__(name, [])
        
        """ List with possible inputs to this state """
        commands = ["STOP"]
        check_stop = CheckState("CheckStop", commands)

        stop_action = StopAction("StopAction")
        
        
        self.add_children([check_stop, stop_action])
        
    def run(self):
        """Access the second element in tuple"""
        return super().run()

if __name__ == "__main__":
    stop = Stop("Stop")
    print(stop.run()[1])