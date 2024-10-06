from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees import logging as log_tree


class StopTree(Behaviour):
    def __init__(self,name="stop"):
        self.counter = 0
        super(StopTree, self).__init__(name)

    def update(self):

        # for robot in :
        #     robot.comand = "stop"
        # self.logger.debug(f"StopTree::update stopped")

        self.counter = self.timer()
        self.logger.debug(f"StopTree::update stop with {self.counter}")
        if self.counter >= 10:
            return Status.SUCCESS
        else:
            return Status.FAILURE            

    def terminate(self, new_status):
        self.logger.debug(f"StopTree::terminate stop to {new_status}")

    def timer(self):
        cont = 0
        for i in range(10):
            cont += 1
        return cont
