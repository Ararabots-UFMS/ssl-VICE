
from strategy.behaviour import LeafNode, Selector, Sequence, TaskStatus
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
    
class CheckIfOurPenalty(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = Blackboard()

    def run(self):
        success = False

        if (self.blackboard.gui.is_team_color_yellow == True) and (self.blackboard.referee.command == "PREPARE_PENALTY_YELLOW"):
            success = True
        elif (self.blackboard.gui.is_team_color_yellow == False) and (self.blackboard.referee.command == "PREPARE_PENALTY_BLUE"):
            success = True
        
        if success:
            return TaskStatus.SUCCESS, "None"
        else:
            return TaskStatus.FAILURE, "None"

class OurPenaltyAction(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        
    def run(self):
        return TaskStatus.SUCCESS, "OUR PENALTY ACTION"
    
class TheirPenaltyAction(LeafNode):
    def __init__(self, name):
        self.name = name
        
    def run(self):
        return TaskStatus.SUCCESS, "THEIR PENALTY ACTION"
    
class Penalty(Sequence):
    def __init__(self, name):
        super().__init__(name, [])
                
        """ List with possible inputs to this state """
        commands = ["PREPARE_PENALTY_BLUE", "PREPARE_PENALTY_YELLOW"]
        check_penalty = CheckState("CheckPenalty", commands)
        
        is_ours = CheckIfOurPenalty("CheckIfOurPenalty")
        action_ours = OurPenaltyAction("OurPenaltyAction")

        ours = Sequence("OurPenalty", [is_ours, action_ours])
        
        action_theirs = TheirPenaltyAction("TheirPenaltyAction")
        
        ours_or_theirs = Selector("OursOrTheirsPenalty", [ours, action_theirs])        
        
        self.add_children([check_penalty, ours_or_theirs])

    def run(self):
        """Access the second element in tuple"""
        return super().run()

if __name__ == "__main__":
    penalty = Penalty("Penalty")
    print(penalty.run()[1])