from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees import logging as log_tree
from system_interfaces.msg import Robots
from py_trees.composites import Sequence
from strategy.blackboard import Blackboard

# Condição simples para exemplo
class Condition(Behaviour):
    def __init__(self):
        self.blackboard = Blackboard()
        super(Condition, self).__init__('Defender_condition')

    def initialise(self):
        for ally_robot in self.blackboard.ally_robots:
            if ally_robot.position_x < 0:
                self.logger.debug(f"Condition::initialise defender")                

    def update(self):
        for ally_robot in self.blackboard.ally_robots:
            ally_robot.behaviour_tree = Defender()  
        

    def terminate(self, new_status):
        self.logger.debug(f"Condition::terminate {self.name} to {new_status}")

class Defender(Behaviour):
    def __init__(self):
        self.blackboard = Blackboard()
        self.position_x = blackboard.ally_robots.position_x
        super(Defender, self).__init__('Defender_action')

    def update(self):
        if position_x >= 0:
            return Status.FAILURE


    def terminate(self, new_status):
        self.logger.debug(f"Condition::terminate {self.name} to {new_status}")

def main():
    root = Sequence(name="root", memory=True)
    attack = Condition()
    root.add_children([
        attack
    ])

if __name__ == "__main__":
    main()


