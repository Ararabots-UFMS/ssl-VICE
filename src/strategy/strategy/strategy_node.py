from time import sleep
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.composites import Selector
from py_trees import logging as log_tree

# classe para verificar se a sub-árvore será executada
class CheckStringCondition(Behaviour):
    def __init__(self, name, input_string):
        super(CheckStringCondition, self).__init__(name)
        self.input_string = input_string

    def initialise(self):
        self.logger.debug(f"CheckStringCondition::initialise {self.name}")

    def update(self):
        self.logger.debug(f"CheckStringCondition::update {self.name}")
        sleep(1)
        # Condição: Se a string for "timeout", retorne FAILURE
        if self.input_string == "timeout":
            self.logger.debug(f"String is 'timeout', returning FAILURE")
            return Status.FAILURE
        self.logger.debug(f"String is '{self.input_string}', returning SUCCESS")
        return Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"CheckStringCondition::terminate {self.name} to {new_status}")

# Ação simples para exemplo
class Action(Behaviour):
    def __init__(self, name, max_attempt_count=1):
        super(Action, self).__init__(name)
        self.max_attempt_count = max_attempt_count
        self.attempt_count = max_attempt_count

    def setup(self):
        self.logger.debug(f"Action::setup {self.name}")

    def initialise(self):
        self.attempt_count = self.max_attempt_count
        self.logger.debug(f"Action::initialise {self.name}")

    def update(self):
        self.attempt_count -= 1
        self.logger.debug(f"Action::update {self.name}")
        sleep(1)
        if not self.attempt_count:
            return Status.SUCCESS

        return Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug(f"Action::terminate {self.name} to {new_status}")

# Condição simples para exemplo
class Condition(Behaviour):
    def __init__(self, name):
        super(Condition, self).__init__(name)

    def setup(self):
        self.logger.debug(f"Condition::setup {self.name}")

    def initialise(self):
        self.logger.debug(f"Condition::initialise {self.name}")

    def update(self):
        self.logger.debug(f"Condition::update {self.name}")
        sleep(1)
        return Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"Condition::terminate {self.name} to {new_status}")



# Função responsável por definir a raiz da árvore
def make_bt(input_string):
    root = Selector(name="root", memory=True)
    timeout_play = Sequence(name="timeout", memory=True)
    stop_play = Sequence(name="stop", memory=True)
    half_play = Sequence(name="half", memory=True)

    # Condição personalizada para verificar a string de entrada
    string_check = CheckStringCondition("Check Input String", input_string)

    # Comportamentos para a sequência
    check_battery = Condition("check_battery")
    open_gripper = Action("open_gripper")
    approach_object = Action("approach_object")
    close_gripper = Action("close_gripper")

    # Adiciona a verificação de string antes de continuar com o 'timeout_play'
    timeout_play.add_children([
        string_check,  # Verifica a string antes de executar os outros nós
        check_battery,
        open_gripper,
        approach_object,
        close_gripper
    ])

    # Subávores
    check_battery1 = Condition("check_battery")
    open_gripper1 = Action("open_gripper", 2)
    approach_object1 = Action("approach_object", 5)
    close_gripper1 = Action("close_gripper", 3)

    stop_play.add_children([
        check_battery1,
        open_gripper1,
        approach_object1,
        close_gripper1
    ])

    check_battery2 = Condition("check_battery")
    open_gripper2 = Action("open_gripper", 2)
    approach_object2 = Action("approach_object", 5)
    close_gripper2 = Action("close_gripper", 3)

    half_play.add_children([
        check_battery2,
        open_gripper2,
        approach_object2,
        close_gripper2
    ])


    # Árvore principal (Play)
    root.add_children([
        timeout_play, 
        stop_play,
        half_play
    ])

    return root

def main():
    log_tree.level = log_tree.Level.DEBUG
    tree = make_bt("timeout") 
    for i in range(1, 20):
        try:
            print("New Tick")
            tree.tick_once()
            sleep(0.1)
        except KeyboardInterrupt:
            break
if __name__ == "__main__":
    main()