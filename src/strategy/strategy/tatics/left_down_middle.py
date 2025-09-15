from strategy.behaviour import Selector, TaskStatus
from strategy.skills.move import MoveSkill

class LeftDownMiddleFormation(Selector):
    def __init__(self, name, robot_id):
        super().__init__(name, [])
        move_skill = MoveSkill(name="tri_r1", robot_id=1, target_x=-1225.0, target_y=-750.0)
        self.moves = [move_skill]
        self.add_children(self.moves)

    def run(self):
        return TaskStatus.RUNNING, list(self.moves)
