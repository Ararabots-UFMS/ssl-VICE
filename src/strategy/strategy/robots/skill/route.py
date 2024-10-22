from control_unit.robot import Robot
from movement.move import Movement
from strategy.blackboard import Blackboard

from strategy.behaviour import LeafNode, Sequence, Selector
from strategy.behaviour import TaskStatus

"""This file should have a properly class to use the Movement classes, look the example below"""

class MoveToPoint(Robot):
    """This class should be a strategy skill to move a robot to a especific point"""
    def __init__(self, name):
        super().__init__(name)
        self.name = name
        self.blackboard = Blackboard()

    def run(self):

        """Moviment to point when the robot is not goalkeeper"""

        if self.blackboard.gui.is_team_color_yellow == True and self.id != self.blackboard.referee._teams[1].goalkeeper:
            return Movement()
        elif self.blackboard.gui.is_team_color_yellow == False and self.id != self.blackboard.referee._teams[0].goalkeeper:
            return Movement()
        
        """Moviment to point when the robot is goalkeeper"""

        if self.blackboard.gui.is_team_color_yellow == True and self.id == self.blackboard.referee._teams[1].goalkeeper:
            return Movement()
        elif self.blackboard.gui.is_team_color_yellow == False and self.id == self.blackboard.referee._teams[0].goalkeeper:
            return Movement()
        
        
            