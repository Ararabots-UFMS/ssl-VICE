from strategy.behaviour import Sequence
from strategy.tatics.go_for_aiming_goal_left import GoForAimingGoalLeft
from strategy.tatics.go_for_aiming_goal_left_kick import GoForAimingGoalLeftKick
from strategy.tatics.go_for_aiming_goal_right import GoForAimingGoalRight
from strategy.tatics.go_for_aiming_goal_right_kick import GoForAimingGoalRightKick
from strategy.tatics.run_of_ball_radius import RunOffBallRadius
from strategy.tatics.aim_to_goal_left import AimToGoalLeft
from strategy.tatics.aim_to_goal_right import AimToGoalRight

from system_interfaces.msg import GameState
import math

class AttackFreekick(Sequence):
    def __init__(self, name):
        super().__init__(name, [])
        self.game_state_sub = self.create_subscription(GameState, 'game_state', self.game_state_callback, 10)
        self.closest_robot_to_ball = self.closest_robot_to_ball()
        self.ally_robots = []
        self.balls = []
        self.is_team_side_left = None
        self.ball = None
        self.robot = None
        self.closest_robot = None
        self.distance = float('inf')
        self.min_distance = float('inf')
        self.children = []


    def game_state_callback(self, msg: GameState):
        self.ally_robots = msg.ally_robot
        self.is_team_side_left = msg.is_team_side_left
        try:
            self.balls = msg.balls
            self.ball = self.balls[0]
        except:
            pass

        self.robot = self.ally_robots.id


    def closest_robot_to_ball(self, ally_robots, ball):
        for robot in ally_robots:
            self.distance = math.sqrt((robot.x - ball.x)**2 + (robot.y - ball.y)**2)
            if self.distance < self.min_distance:
                self.min_distance = self.distance
                self.closest_robot = ally_robots[robot.id]
        return self.closest_robot
    
    def update_robots_state(self, closest_robot, is_team_side_left):
        if is_team_side_left == True:
            for robot in self.ally_robots:
                if robot.id == closest_robot.id:
                    self.children.append(GoForAimingGoalLeft(closest_robot, closest_robot.x, closest_robot.y, self.ball.x, self.ball.y, goal_x=2250, goal_y=0, robot_radius=85))
                    self.children.append(GoForAimingGoalLeftKick(closest_robot, closest_robot.x, closest_robot.y, self.ball.x, self.ball.y, goal_x=2250, goal_y=0, robot_radius=85))
                else:
                    self.children.append(RunOffBallRadius(self.ally_robots[robot.id], self.ally_robots[robot.id].x, self.ally_robots[robot.id].y, self.ball.x, self.ball.y, robot_radius=85))
                    self.children.append(AimToGoalLeft(self.ally_robots[robot.id], self.ally_robots[robot.id].orientation, goal_x=2250, goal_y=0))
        else:
            for robot in self.ally_robots:
                if robot.id == closest_robot.id:
                    self.children.append(GoForAimingGoalRight(closest_robot, closest_robot.x, closest_robot.y, self.ball.x, self.ball.y, goal_x=-2250, goal_y=0, robot_radius=85))
                    self.children.append(GoForAimingGoalRightKick(closest_robot, closest_robot.x, closest_robot.y, self.ball.x, self.ball.y, goal_x=-2250, goal_y=0, robot_radius=85))
                else:
                    self.children.append(RunOffBallRadius(self.ally_robots[robot.id], self.ally_robots[robot.id].x, self.ally_robots[robot.id].y, self.ball.x, self.ball.y, robot_radius=85))
                    self.children.append(AimToGoalRight(self.ally_robots[robot.id], self.ally_robots[robot.id].orientation, goal_x=-2250, goal_y=0))

    def run(self):
        return super().run()