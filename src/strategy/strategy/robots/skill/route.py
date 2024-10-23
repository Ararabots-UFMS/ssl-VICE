from movement.move import Movement
from strategy.blackboard import Blackboard


"""This file should have a properly class to use the Movement classes, look the example below"""

class MoveToPoint():
    """This class should be a strategy skill to move a robot to a especific point"""
    def __init__(self,name):
        super().__init__()
        self.name = name
        self.id = 1
        self.blackboard = Blackboard()

    def run(self):

        """Moviment to point when the robot is not goalkeeper"""

        if self.blackboard.gui.is_team_color_yellow == True and self.id != self.blackboard.referee._teams[1].goalkeeper:
            # return Movement(self.id)
            return self.name
        elif self.blackboard.gui.is_team_color_yellow == False and self.id != self.blackboard.referee._teams[0].goalkeeper:
            # return Movement(self.id)
            return self.name
        
        """Moviment to point when the robot is goalkeeper"""

        if self.blackboard.gui.is_team_color_yellow == True and self.id == self.blackboard.referee._teams[1].goalkeeper:
            # return Movement(self.id)
            return self.name
        elif self.blackboard.gui.is_team_color_yellow == False and self.id == self.blackboard.referee._teams[0].goalkeeper:
            # return Movement(self.id)
            return self.name
        
        
            