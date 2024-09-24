import time
from strategy.state_machine import StateMachine, create_blackboard
from strategy.strategy_node import make_bt
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees import logging as log_tree



limit_time = 1.0 #milisseconds

# Function to simulate the state machine and tick the behavior tree
def test_time_stop():
    # Tick the behavior tree with the updated state
    start_time = time.time()
    tree = make_bt("stop")  # Update the tree with the new state
    tree.tick_once()
    end_time = time.time()

    # Calculate and display execution time
    total_time = (end_time - start_time) * 1000

    assert total_time <= limit_time        

def test_time_half():
    # Tick the behavior tree with the updated state
    start_time = time.time()
    tree = make_bt("half")  # Update the tree with the new state
    tree.tick_once()
    end_time = time.time()

    # Calculate and display execution time
    total_time = (end_time - start_time) * 1000

    assert total_time <= limit_time        

def test_time_timeout():
    # Tick the behavior tree with the updated state
    start_time = time.time()
    tree = make_bt("timeout")  # Update the tree with the new state
    tree.tick_once()
    end_time = time.time()

    # Calculate and display execution time
    total_time = (end_time - start_time) * 1000

    assert total_time <= limit_time        




