from strategy.behaviour import BaseTree

from strategy.coach.kickoff import Kickoff

class CoachStrategy(BaseTree):
    def __init__(self):
        super().__init__()
        self._root = None

    def set_root(self, root):
        self._root = root

    def execute(self):
        self._root.execute()