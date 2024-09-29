from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.composites import Selector
from py_trees import logging as log_tree
import time

class Action(Behaviour):
    def __init__(self, name, comand):
        super(Condition, self).__init__(name)
        self.comand = comand

    def initialise(self):
        self.logger.debug(f"Condition::initialise {self.name}")

    def update(self):
        self.logger.debug(f"Condition::update {self.name}")
        # Condição: Se a string de entrada for igual ao nome do comportamento (subárvore)
        if self.comand == self.name:
            self.logger.debug(f"String matches '{self.name}', returning SUCCESS")
            return Status.SUCCESS
        self.logger.debug(f"String is '{self.comand}', does not match '{self.name}', returning FAILURE")
        return Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug(f"Condition::terminate {self.name} to {new_status}")


# Condição simples para exemplo
class Condition(Behaviour):
    def __init__(self, name, comand):
        super(Condition, self).__init__(name)
        self.comand = comand

    def initialise(self):
        self.logger.debug(f"Condition::initialise {self.name}")

    def update(self):
        self.logger.debug(f"Condition::update {self.name}")
        # Condição: Se a string de entrada for igual ao nome do comportamento (subárvore)
        if self.comand == self.name:
            self.logger.debug(f"String matches '{self.name}', returning SUCCESS")
            return Status.SUCCESS
        self.logger.debug(f"String is '{self.comand}', does not match '{self.name}', returning FAILURE")
        return Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug(f"Condition::terminate {self.name} to {new_status}")


# Função responsável por definir a raiz da árvore
def make_bt(comand):
    root = Selector(name="root", memory=True)
    timeout_play = Sequence(name="timeout", memory=True)
    stop_play = Sequence(name="stop", memory=True)
    half_play = Sequence(name="half", memory=True)

    # Condição personalizada para verificar a string de entrada
    check_timeout = Condition("timeout", comand)
    check_stop = Condition("stop", comand)
    check_half = Condition("half", comand)

   

    # Adiciona a verificação de string antes de continuar com o 'timeout_play'
    timeout_play.add_children([
        check_timeout,
        
    ])

    stop_play.add_children([
        check_stop,
        
    ])

    half_play.add_children([
        check_half,
        
    ])

    # Árvore principal
    root.add_children([
        timeout_play, 
        stop_play,
        half_play,
    ])

    return root

def main():
    log_tree.level = log_tree.Level.DEBUG
    inicio = time.time()
    tree = make_bt("half") 
    print("New Tick")
    tree.tick_once()
    fim = time.time()
    tempo_total = fim - inicio
    print(f"Tempo de execução: {tempo_total * 1000} milissegundos")

if __name__ == "__main__":
    main()
