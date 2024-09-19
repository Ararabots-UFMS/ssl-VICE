import py_trees

# Blackboard client setup
def create_blackboard():
    blackboard_client = py_trees.blackboard.Client(name="State Client", namespace="state_machine_")
    blackboard_client.register_key("current_state", access=py_trees.common.Access.WRITE)
    blackboard_client.current_state = "half"  # Initial state
    return blackboard_client