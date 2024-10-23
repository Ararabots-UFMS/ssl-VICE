
from strategy.blackboard import Blackboard
from strategy.robots.skill.route import MoveToPoint

"""Contains all KickOffActions the robot must do (in order or not) during the match"""

class OurAttackerAction():
    def __init__(self, name):
        self.name = name
        self.blackboard = Blackboard()
        self.movement = MoveToPoint(self.name)

    def run(self):
        return self.movement.run()


class TheirAttackerAction():
    def __init__(self, name):
        self.name = name
        self.blackboard = Blackboard()
        self.movement = MoveToPoint(self.name)

    def run(self):
        return self.movement.run()