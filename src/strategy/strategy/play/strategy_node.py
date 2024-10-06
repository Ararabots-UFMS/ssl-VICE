from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.composites import Selector
from py_trees import logging as log_tree
from strategy.play.halt_tree import HaltTree
from strategy.play.stop_tree import StopTree
from strategy.play.timeout_tree import TimeoutTree
from strategy.play.base_tree import Condition
import time

# Função responsável por definir a raiz da árvore
def make_bt(input_string):
    root = Selector(name="root", memory=True)
    timeout_play = Sequence(name="timeout", memory=True)
    stop_play = Sequence(name="stop", memory=True)
    halt_play = Sequence(name="halt", memory=True)

    # Comportamentos para a sequência
    check_timeout = Condition("timeout", input_string)
    timeout_action = TimeoutTree()

    # Comportamentos para a sequência
    check_stop = Condition("stop", input_string)
    stop_action = StopTree()

    # Comportamentos para a sequência
    check_halt = Condition("halt", input_string)
    halt_action = HaltTree()

    # Adiciona a verificação de string antes de continuar com o 'timeout_play'
    timeout_play.add_children([
        check_timeout,
        timeout_action
    ])

    stop_play.add_children([
        check_stop,
        stop_action
    ])

    halt_play.add_children([
        check_halt,
        halt_action
    ])

    # Árvore principal
    root.add_children([
        timeout_play, 
        stop_play,
        halt_play,
    ])

    return root