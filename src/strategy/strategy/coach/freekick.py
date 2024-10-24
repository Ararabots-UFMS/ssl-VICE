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
    
class CheckIfOurFreeKick(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = Blackboard()

    def run(self):
        success = False

        if (self.blackboard.gui.is_team_color_yellow == True) and (self.blackboard.referee.command == "DIRECT_FREE_YELLOW"):
            success = True
        elif (self.blackboard.gui.is_team_color_yellow == False) and (self.blackboard.referee.command == "DIRECT_FREE_BLUE"):
            success = True
        
        if success:
            return TaskStatus.SUCCESS, "None"
        else:
            return TaskStatus.FAILURE, "None"

class OurFreekickAction(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        
    def run(self):
        return TaskStatus.SUCCESS, "OUR FREE-KICK"
    
class TheirFreeKickAction(LeafNode):
    def __init__(self, name):
        self.name = name
        
    def run(self):
        return TaskStatus.SUCCESS, "THEIR FREE-KICK"
    
class FreeKick(Sequence):
    def __init__(self, name):
        super().__init__(name, [])
        
        """ List with possible inputs to this state """
        commands = ["DIRECT_FREE_BLUE", "DIRECT_FREE_YELLOW"]
        check_freekick = CheckState("CheckFreeKick", commands)
        
        is_ours = CheckIfOurFreeKick("CheckIfOurFreeKick")
        action_ours = OurFreekickAction("OurFreekickAction")

        ours = Sequence("OurFreeKick", [is_ours, action_ours])
        
        action_theirs = TheirFreeKickAction("TheirFreeKickAction")
        
        ours_or_theirs = Selector("OursOrTheirsFreeKick", [ours, action_theirs])        
        
        self.add_children([check_freekick, ours_or_theirs])
        
    def run(self):
        """Access the second element in tuple"""
        return super().run()

if __name__ == "__main__":
    freekick = FreeKick("FreeKick")
    print(freekick.run()[1])