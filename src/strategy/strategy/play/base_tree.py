from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees import logging as log_tree


# Condição simples para exemplo
class Condition(Behaviour):
    def __init__(self, name, input_string):
        super(Condition, self).__init__(name)
        self.input_string = input_string

    def initialise(self):
        self.logger.debug(f"Condition::initialise {self.name}")

    def update(self):
        self.logger.debug(f"Condition::update {self.name}")
        # Condição: Se a string de entrada for igual ao nome do comportamento (subárvore)
        if self.input_string == self.name:
            self.logger.debug(f"String matches '{self.name}', returning SUCCESS")
            return Status.SUCCESS
        self.logger.debug(f"String is '{self.input_string}', does not match '{self.name}', returning FAILURE")
        return Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug(f"Condition::terminate {self.name} to {new_status}")

