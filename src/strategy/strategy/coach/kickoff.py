from strategy.blackboard import Blackboard

from strategy.behaviour import LeafNode, Sequence, Selector
from strategy.behaviour import TaskStatus

class CheckState(LeafNode):
    def __init__(self, name, _desired_states):
        self.name = name
        self.blackboard = Blackboard()
        self.desired_states = _desired_states

    def run(self):
        print(self.name, None)
        if self.blackboard.referee.command in self.desired_states:
            return TaskStatus.SUCCESS, "None"

        return TaskStatus.FAILURE, "None"
    
class CheckIfOurKickoff(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = Blackboard()

    def run(self):
        print(self.name, None)
        success = False
        
        if self.blackboard.gui.is_team_color_yellow == True and self.blackboard.referee.command == "PRAPERE_KICKOFF_YELLOW":
            success = True
        elif self.blackboard.gui.is_team_color_yellow == False and self.blackboard.referee.command == "PRAPERE_KICKOFF_BLUE":
            success = True
            
        if success:
            return TaskStatus.SUCCESS, "None"
        else:
            return TaskStatus.FAILURE, "None"

class OurKickoffAction(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        
    def run(self):
        print(self.name, "OUR KICK OFF")
        return TaskStatus.SUCCESS, "OUR KICK OFF"
    
class TheirKickoffAction(LeafNode):
    def __init__(self, name):
        self.name = name
        
    def run(self):
        print(self.name, "THEIR KICK OFF")
        return TaskStatus.SUCCESS, "THEIR KICK OFF"
    
class Kickoff(Sequence):
    def __init__(self, name):
        super().__init__(name, [])
        
        commands = ["PREPARE_KICKOFF_BLUE", "PREPARE_KICKOFF_YELLOW"]
        check_kickoff = CheckState("    CheckKickoff", commands)
        
        is_ours = CheckIfOurKickoff("            CheckIfOurKickoff")
        action_ours = OurKickoffAction("            OurKickoffAction")

        ours = Sequence("        OurKickoff", [is_ours, action_ours])
        
        action_theirs = TheirKickoffAction("        TheirKickoffAction")
        
        ours_or_theirs = Selector("    OursOrTheirsKickoff", [ours, action_theirs])        
        
        self.add_children([check_kickoff, ours_or_theirs])
        
    def run(self):
        print(self.name)
        super().run()

if __name__ == "__main__":
    kickoff = Kickoff("Kickoff")
    print(kickoff.run())