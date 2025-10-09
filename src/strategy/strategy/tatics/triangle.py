from strategy.behaviour import LeafNode, TaskStatus
from strategy.skills.skills import Skills

class TriangleFormation(LeafNode):
    def __init__(self, name):
        super().__init__(name)

        ## É SÓ UM EXEMPLO DE COMO USAR A CLASSE SKILLS, NÃO FAZ TRIANGULO DE VERDADE

        skills_factory = Skills("Movement")
        # Robô 0: vai para o centro
        r0 = skills_factory.move_to(robot_id=0, target_x=0.0, target_y=0.0, vel_x=0.0, vel_y=0.0)
        r0.field_border = True

        # # Robô 1: tentativa de ir ao centro mas evitando área central
        r1 = skills_factory.move_to(robot_id=1, target_x=10.0, target_y=0.0, vel_x=0.0, vel_y=0.0)
        r1.center_area = True
        r1.field_border = True

        # Robô 2: vai para o gol adversário evitando área de penalti e orientado para o gol
        enemy_goal_x = 2200.0
        enemy_goal_y = 0.0
        r2 = skills_factory.move_with_angle(
            robot_id=2,
            target_x=enemy_goal_x,
            target_y=enemy_goal_y,
            angle=3.14,
            vel_x=0.0,
            vel_y=0.0,
        )

        r2.penalty_area = True
        r2.field_border = True

        self.robots_action = [r0, r1, r2]

    def run(self):
        return TaskStatus.SUCCESS, list(self.robots_action)
