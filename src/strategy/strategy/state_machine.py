import py_trees
from strategy.black_board import create_blackboard

# State machine implementation
class StateMachine:
    def __init__(self):
        self.states = ["half", "stop", "timeout"]
        self.current_state_index = 0

    def get_next_state(self):
        # Cycle through states: half -> stop -> timeout -> half
        self.current_state_index = (self.current_state_index + 1) % len(self.states)
        return self.states[self.current_state_index]
