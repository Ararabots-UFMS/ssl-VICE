from strategy.behaviour import Sequence
from strategy.tatics.defend_goal_left import DefendGoalLeft
from strategy.tatics.defend_goal_right import DefendGoalRight
from strategy.tatics.defend_area_left import DefendAreaLeft
from strategy.tatics.defend_area_right import DefendAreaRight
from strategy.tatics.defend_area_left_down import DefendAreaLeftDown
from strategy.tatics.defend_area_right_down import DefendAreaRightDown

from system_interfaces.msg import GameState
class DefenseFreekick(Sequence):
    def __init__(self, name):
        super().__init__(name, [])
        self.game_state_sub = self.create_subscription(GameState, 'game_state', self.game_state_callback, 10)
        self.ally_robots = []
        self.balls = []
        self.is_team_side_left = None
        self.ball = None
        self.robot = None
        self.goalkeeper = None
        self.defender = None
        self.children = []

    def game_state_callback(self, msg: GameState):
        self.ally_robots = msg.ally_robot
        self.is_team_side_left = msg.is_team_side_left
        try:
            self.balls = msg.balls
            self.ball = self.balls[0]
        except:
            pass

        try:
            self.goalkeeper = self.ally_robots[2]
            self.defender = self.ally_robots[0]
        except:
            pass

        self.robot = self.ally_robots.id


    def update_robot_state(self, ally_robots, goalkeeper, defender, is_team_side_left):
        if is_team_side_left == True:
            for robot in ally_robots:
                if robot.id == goalkeeper.id:
                    self.children.append(DefendGoalLeft(goalkeeper, goalkeeper.x, goalkeeper.y, self.ball.x, self.ball.y, goal_x=2250, goal_y=0))
                elif robot.id == defender.id:
                    self.children.append(DefendAreaLeft(defender, robot.x, robot.y, self.ball.x, self.ball.y, goal_x=2250, goal_y=0, robot_radius=85))
                else:
                    self.children.append(DefendAreaLeftDown(robot, robot.x, robot.y, self.ball.x, self.ball.y, goal_x=2250, goal_y=0, robot_radius=85, distance=5))
        else:
            for robot in ally_robots:
                if robot.id == goalkeeper.id:
                    self.children.append(DefendGoalRight(goalkeeper, goalkeeper.x, goalkeeper.y, self.ball.x, self.ball.y, goal_x=-2250, goal_y=0))
                elif robot.id == defender.id:
                    self.children.append(DefendAreaRight(defender, robot.x, robot.y, self.ball.x, self.ball.y, goal_x=-2250, goal_y=0, robot_radius=85))
                else:
                    self.children.append(DefendAreaRightDown(robot, robot.x, robot.y, self.ball.x, self.ball.y, goal_x=-2250, goal_y=0, robot_radius=85, distance=5))