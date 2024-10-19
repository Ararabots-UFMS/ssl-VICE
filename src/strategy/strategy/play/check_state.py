from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees import logging as log_tree


# Condição simples para exemplo
class CheckState(Behaviour):
    def __init__(self, name, commmands ,blackboard):
        super(CheckState, self).__init__(name)
        self.blackboard = blackboard
        self.commands = commmands

    def initialise(self):
        self.logger.debug(f"CheckState::initialise {self.name}")

    def update(self):
        self.logger.debug(f"CheckState::update {self.name}")
        # Condição: Se a string de entrada for igual ao nome do comportamento (subárvore)
        if self.blackboard.referee.command in self.commands:
            self.logger.debug(f"String matches '{self.name}', returning SUCCESS")
            return Status.SUCCESS
        self.logger.debug(f"String is '{self.blackboard.referee.command}', does not match '{self.name}', returning FAILURE")
        return Status.FAILURE

    def terminate(self, new_status):

        self.logger.debug(f"CheckState::terminate {self.name} to {new_status}")

