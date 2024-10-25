from strategy.blackboard import Blackboard

from strategy.behaviour import LeafNode, Sequence, Selector
from strategy.behaviour import TaskStatus

class CheckState(LeafNode):
    def __init__(self, name, _desired_states):
        self.name = name
        self.blackboard = Blackboard()
        self.desired_states = _desired_states

    def run(self):
        if self.blackboard.referee.command in self.desired_states:
            return TaskStatus.SUCCESS, "None"

        return TaskStatus.FAILURE, "None"
    
class _CheckIfOurTimeout(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = Blackboard()

    def run(self):
        success = False

        if (self.blackboard.gui.is_team_color_yellow == True) and (self.blackboard.referee.command == "TIMEOUT_YELLOW"):
            success = True
        elif (self.blackboard.gui.is_team_color_yellow == False) and (self.blackboard.referee.command == "TIMEOUT_BLUE"):
            success = True
        
        if success:
            return TaskStatus.SUCCESS, "None"
        else:
            return TaskStatus.FAILURE, "None"

class OurTimeoutAction(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        
    def run(self):
        return TaskStatus.SUCCESS, "OUR TIMEOUT"
    
class TheirTimeoutAction(LeafNode):
    def __init__(self, name):
        self.name = name
        
    def run(self):
        return TaskStatus.SUCCESS, "THEIR TIMEOUT"
    
class _Timeout(Sequence):
    def __init__(self, name):
        super().__init__(name, [])
        
        """ List with possible inputs to this state """
        commands = ["TIMEOUT_BLUE", "TIMEOUT_YELLOW"]
        check_timeout = CheckState("CheckTimeout", commands)
        
        is_ours = _CheckIfOurTimeout("CheckIfOurTimeout")
        action_ours = OurTimeoutAction("OurTimeoutAction")

        ours = Sequence("OurFreeKick", [is_ours, action_ours])
        
        action_theirs = TheirTimeoutAction("TheirTimeoutAction")
        
        ours_or_theirs = Selector("OursOrTheirsTimeout", [ours, action_theirs])        
        
        self.add_children([check_timeout, ours_or_theirs])
        
    def run(self):
        """Access the second element in tuple"""
        return super().run()

if __name__ == "__main__":
    timeout = _Timeout("Timeout")
    print(timeout.run()[1])