import time
from strategy.state_machine import StateMachine, create_blackboard
from strategy.strategy_node import make_bt
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees import logging as log_tree
        
# Function to simulate the state machine and tick the behavior tree
def run_state_machine_and_tick(tree, blackboard_client, state_machine, iterations=3):
    for i in range(iterations):
        # Get the next state from the state machine
        next_state = state_machine.get_next_state()
        # Update the blackboard with the new state
        blackboard_client.current_state = next_state
        print(f"Blackboard State updated to: {blackboard_client.current_state}")

        # Tick the behavior tree with the updated state
        print("New Tick")
        start_time = time.time()
        tree = make_bt(blackboard_client.current_state)  # Update the tree with the new state
        tree.tick_once()
        end_time = time.time()

        # Calculate and display execution time
        total_time = (end_time - start_time) * 1000
        print(f"Execution time: {total_time:.2f} ms")

# Main function to run the state machine and tick the behavior tree
def main():
    # Setup blackboard and state machine
    blackboard_client = create_blackboard()
    state_machine = StateMachine()
    log_tree.level = log_tree.Level.DEBUG
    # Create the initial tree based on the blackboard state
    tree = make_bt(blackboard_client.current_state)
    
    # Run the state machine and tick the behavior tree
    run_state_machine_and_tick(tree, blackboard_client, state_machine)

if __name__ == "__main__":
    main()
