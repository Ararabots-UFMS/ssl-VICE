import py_trees

# State machine implementation
class StateMachine:
    def __init__(self):
        self.states = ["half", "stop", "timeout"]
        self.current_state_index = 0

    def get_next_state(self):
        # Cycle through states: half -> stop -> timeout -> half
        self.current_state_index = (self.current_state_index + 1) % len(self.states)
        return self.states[self.current_state_index]

# Blackboard client setup
def create_blackboard():
    blackboard_client = py_trees.blackboard.Client(name="State Client", namespace="state_machine_")
    blackboard_client.register_key("current_state", access=py_trees.common.Access.WRITE)
    blackboard_client.current_state = "half"  # Initial state
    return blackboard_client
