from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.composites import Selector
from py_trees import logging as log_tree
from strategy.play.halt_tree import HaltTree
from strategy.play.kick_off_tree import KickOff
from strategy.play.stop_tree import StopTree
from strategy.play.timeout_tree import TimeoutTree
from strategy.play.check_state import CheckState
from strategy.blackboard import Blackboard
import time

# Função responsável por definir a raiz da árvore
def make_bt():
    blackboard = Blackboard()
    root = Selector(name="root", memory=True)
    timeout_play = Sequence(name="TIMEOUT", memory=True)
    stop_play = Sequence(name="STOP", memory=True)
    halt_play = Sequence(name="HALF", memory=True)
    kick_off_play = Sequence(name="kick-off", memory=True)

    # Comportamentos para a sequência
    check_timeout = CheckState("TIMEOUT", ["TIMEOUT"], blackboard)
    timeout_action = TimeoutTree()

    # Comportamentos para a sequência
    check_stop = CheckState("STOP", ["STOP"],blackboard)
    stop_action = StopTree()

    # Comportamentos para a sequência
    check_halt = CheckState("HALT", ["HALT"], blackboard)
    halt_action = HaltTree()

    check_kick_off = CheckState("PREPARE_KICKOFF",["PREPARE_KICKOFF_BLUE","PREPARE_KICKOFF_YELLOW"] ,blackboard)
    kick_off_action = KickOff()

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

    kick_off_play.add_children([
        check_kick_off,
        kick_off_action
    ])

    # Árvore principal
    root.add_children([
        timeout_play, 
        stop_play,
        halt_play,
        kick_off_play
    ])

    return root