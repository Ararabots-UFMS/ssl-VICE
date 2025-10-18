from strategy.behaviour import LeafNode, TaskStatus
from strategy.skills.skills import Skills
from system_interfaces.msg import GameState

class TriangleFormation(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.create_subscription(GameState, "game_state", self.game_state_callback, 10)
        self.ally_robots = {}

        self.skills_factory = Skills("Movement")
        
    def game_state_callback(self, msg):
        self.ally_robots = {r.id: r for r in msg.ally_robots}

    def execute(self):

        robot_0 = self.ally_robots.get(0)

        if not robot_0:
            return []
        
        target_x = None

        limiar_x = 2000


        edge = 2250.0
        step = 500.0
        
        if not hasattr(self, "patrol_dir"):
            self.patrol_dir = 1 if robot_0.position_x < 0 else -1

        
        if robot_0.position_x >= edge:
            target_x = -edge
            self.patrol_dir = -1
        elif robot_0.position_x <= -edge:
            target_x = edge
            self.patrol_dir = 1
        else:
        
            next_x = robot_0.position_x + self.patrol_dir * step
            if next_x > edge:
                target_x = edge
                self.patrol_dir = -1
            elif next_x < -edge:
                target_x = -edge
                self.patrol_dir = 1
            else:
                target_x = next_x

        
        robot_command = self.skills_factory.move_to(
            robot_id=0,
            target_x=target_x,
            target_y=0,
            vel_x=0.0,
            vel_y=0.0
        )
        
        if target_x is None:
            return None
        return [robot_command]


    def run(self):
        return TaskStatus.SUCCESS, self.execute()
