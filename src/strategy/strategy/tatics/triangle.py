from strategy.behaviour import Selector, TaskStatus
from strategy.skills.move import MoveSkill

class TriangleFormation(Selector):
    def __init__(self, name):
        super().__init__(name, [])
        move_skill = MoveSkill("MoveSkill", robot_id=0, target_x=0.0, target_y=0.0, vel_x=0.0, vel_y=0.0)
        move_skill_2 = MoveSkill("MoveSkill2", robot_id=1, target_x=-1200.0, target_y=1200.0, vel_x=0.0, vel_y=0.0)
        move_skill_3 = MoveSkill("MoveSkill3", robot_id=2, target_x=-1200.0, target_y=-1200.0, vel_x=0.0, vel_y=0.0)

        self.moves = [move_skill, move_skill_2, move_skill_3]
        self.add_children(self.moves)

    def run(self):
        return TaskStatus.RUNNING, list(self.moves)
