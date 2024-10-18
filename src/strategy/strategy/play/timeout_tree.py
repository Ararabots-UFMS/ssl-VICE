from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees import logging as log_tree


class TimeoutTree(Behaviour):
    def __init__(self,name="timeout"):
        self.counter = 0
        super(TimeoutTree, self).__init__(name)

    def setup(self):
        self.logger.debug(f"TimeoutTree::setup timeout")

    def initialise(self):
        # for robots in blackboard.ally_robots:
        #     robots = reduce2stop()
        self.counter = self.timer()

    def update(self):
        
        self.logger.debug(f"TimeoutTree::update timeout with {self.counter}")
        if self.counter >= 10:
            return Status.SUCCESS
        else:
            return Status.FAILURE            

    def terminate(self, new_status):
        self.logger.debug(f"TimeoutTree::terminate timeout to {new_status}")

    def timer(self):
        cont = 0
        for i in range(10):
            cont += 1
        return cont
