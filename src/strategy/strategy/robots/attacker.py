from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees import logging as log_tree
from system_interfaces.msg import Robots

# Condição simples para exemplo
class Condition(Behaviour):
    def __init__(self):
        self.blackboard = Blackboard()
        super(Condition, self).__init__('attacker_condition')

    def initialise(self):
        for ally_robot in self.blackboard.ally_robots:
            if ally_robot.position_x >= 0:
                self.logger.debug(f"Condition::initialise atacante")                

    def update(self):
        for ally_robot in self.blackboard.ally_robots:
            ally_robot.behaviour_tree = Attacker()  
        

    def terminate(self, new_status):
        self.logger.debug(f"Condition::terminate {self.name} to {new_status}")

class Attacker(Behaviour):
     def __init__(self):
        self.blackboard = Blackboard()
        self.position_x = blackboard.ally_robots.position_x
        super(Attacker, self).__init__('attacker_action')

    def update(self):
        if position_x < 0
        return Status.FAILURE


    def terminate(self, new_status):
        self.logger.debug(f"Condition::terminate {self.name} to {new_status}")


