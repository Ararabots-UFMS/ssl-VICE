import time
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees import logging as log_tree
from strategy.blackboard import Blackboard
from strategy.play.strategy_node import make_bt
        

# Main function to run the state machine and tick the behavior tree
def main(args=None):
    blackboard = Blackboard()
    log_tree.level = log_tree.Level.DEBUG
    # Setup blackboard and state machine
    # Create the initial tree based on the blackboard state
    print("New Tick")
    start_time = time.time()
    tree = make_bt(blackboard.referee.command)  # Update the tree with the new state
    tree.tick_once()
    end_time = time.time()

    # Calculate and display execution time
    total_time = (end_time - start_time) * 1000
    print(f"Execution time: {total_time:.2f} ms")
    

if __name__ == "__main__":
    main()