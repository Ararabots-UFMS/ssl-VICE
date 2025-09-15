from strategy.blackboard import Blackboard
from strategy.behaviour import Selector
from strategy.tatics.center_area import CenterAreaFormation
from strategy.tatics.left_down_middle import LeftDownMiddleFormation
from strategy.tatics.left_up_middle import LeftUpMiddleFormation

class KickOff(Selector):
    def __init__(self, name):
        super().__init__(name, [])

        robot_id_center = self.get_closest_robot_to_center()
        robot_id_bottom = self.get_closest_robot_to_bottom_touch_line()
        robot_id_top = self.get_closest_robot_to_top_touch_line()

        center_area_formation = CenterAreaFormation("CenterAreaFormation", robot_id_center)
        bottom_touch_line = LeftDownMiddleFormation("BottomTouchLine", robot_id_bottom)
        top_touch_line = LeftUpMiddleFormation("TopTouchLine", robot_id_top)

        self.add_children([center_area_formation, bottom_touch_line, top_touch_line])

    def get_closest_robot_to_center(self):
        blackboard = Blackboard()
        min_dist = float('inf')
        closest_robot_to_center = None
        for robot in blackboard.ally_robots.values():
            dist = (abs((robot.x - -250)) ** 2 + abs((robot.y - 0)) ** 2) ** 0.5
            if dist < min_dist:
                min_dist = dist
                closest_robot_to_center = robot
        return closest_robot_to_center.id

    def get_closest_robot_to_bottom_touch_line(self):
        blackboard = Blackboard()
        min_dist = float('inf')
        closest_robot_to_bottom_touch_line = None
        for robot in blackboard.ally_robots.values():
            dist = float(abs(robot.y - -1500))
            if dist < min_dist:
                min_dist = dist
                closest_robot_to_bottom_touch_line = robot
        return closest_robot_to_bottom_touch_line.id

    def get_closest_robot_to_top_touch_line(self):
        blackboard = Blackboard()
        min_dist = float('inf')
        closest_robot_to_top_touch_line = None
        for robot in blackboard.ally_robots.values():
            dist = float(abs(robot.y - 1500))
            if dist < min_dist:
                min_dist = dist
                closest_robot_to_top_touch_line = robot
    
    def run(self):
        return super.run()