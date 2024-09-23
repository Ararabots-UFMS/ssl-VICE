import numpy as np
from movement.obstacles.interfaces import StaticObstacle

from system_interfaces.msg import VisionGeometry


# This needs to change to use the FieldLineSegment instead of the distances
class BoundaryObstacles(StaticObstacle):
    def __init__(self, geometry: VisionGeometry):
        self.half_width = geometry.field_width / 2
        self.half_length = geometry.field_length / 2
    
    def is_colission(self, x: np.matrix, ignore: bool = False, padding: float = 90):
        if ignore:
            return False

        elif x[0] < -1 * (self.half_length - padding) or x[0] > self.half_length - padding:
            return True

        elif x[1] < -1 * (self.half_width - padding) or x[1] > self.half_width - padding:
            return True

        else:
            return False

    def closest_outside_point(self, x: np.matrix, offset: float = 90):
        x_distance = 0
        y_distance = 0
        if x[0] < -1 * self.half_length or x[0] > self.half_length:
            x_distance  = x[0] - self.half_length + offset

        if x[1] < -1 * self.half_width or x[1] > self.half_width:
            y_distance  = x[1] - self.half_width + offset

        if x_distance < y_distance:
            x_distance = 0
        else:
            y_distance = 0

        # Huge mamaco, there may be a better way...
        return np.array([x[0] + (-1 * np.sign(x[0]) * x_distance), x[1] + (-1 * np.sign(x[1]) * y_distance)])

class WallObstacles(StaticObstacle):
    def __init__(self, geometry: VisionGeometry):
        self.half_width = geometry.field_width / 2
        self.half_length = geometry.field_length / 2   
        self.boundary_width = geometry.boundary_width

    def is_colission(self, x: np.matrix, ignore: bool = False, padding: float = 180):
        if ignore:
            return False

        elif x[0] < -1 * (self.half_length + self.boundary_width - padding) or x[0] > self.half_length + self.boundary_width - padding:
            return True

        elif x[1] < -1 * (self.half_width + self.boundary_width - padding) or x[1] > self.half_width + self.boundary_width - padding:
            return True

        else:
            return False

    def closest_outside_point(self, x: np.matrix):
        x_distance = 0
        y_distance = 0
        if x[0] < -1 * self.half_length + self.boundary_width or x[0] > self.half_length + self.boundary_width:
            x_distance  = x[0] - self.half_length - self.boundary_width

        if x[1] < -1 * self.half_width or x[1] > self.half_width:
            y_distance  = x[1] - self.half_width - self.boundary_width

        if x_distance < y_distance:
            x_distance = 0
        else:
            y_distance = 0

        # Huge mamaco, there may be a better way...
        return np.array([x[0] + (-1 * np.sign(x[0]) * x_distance + offset), x[1] + (-1 * np.sign(x[1]) * y_distance)])


class PenaltyAreaObstacles(StaticObstacle):
    def __init__(self, geometry: VisionGeometry):
        for line in geometry.field_lines:

            if line.name == 'LeftGoalLine':
                self.left_goal = line

            elif line.name == 'RightGoalLine':
                self.right_goal = line

            elif line.name == 'LeftPenaltyStretch':
                self.left_penalty = line

            elif line.name == 'RightPenaltyStretch':
                self.right_penalty = line

            elif line.name == 'LeftFieldLeftPenaltyStretch':
                self.left_field_left_penalty = line

            elif line.name == 'LeftFieldRightPenaltyStretch':
                self.left_field_right_penalty = line

            elif line.name == 'RightFieldLeftPenaltyStretch':
                self.right_field_left_penalty = line

            elif line.name == 'RightFieldRightPenaltyStretch':
                self.right_field_right_penalty = line

    def is_colission(self, x: np.matrix, ignore: bool = False, padding: float = 90):
        if ignore:
            return False

        # For the left field side
        # -4500 1000
        if x[0] > self.left_field_left_penalty.x1 and x[0] < self.left_field_left_penalty.x2 + padding:
            if x[1] < self.left_field_left_penalty.y1 + padding and x[1] > self.left_field_right_penalty.y1 - padding:
                return True

        # For the right field side
        if x[0] < self.right_field_left_penalty.x1 and x[0] > self.right_field_left_penalty.x2 - padding:
            if x[1] > self.right_field_left_penalty.y1 - padding and x[1] < self.right_field_right_penalty.y1 + padding:
                return True

        return False

    def closest_outside_point(self, x: np.matrix, offset: float = 90):
        x_distance = 0
        y_distance = 0

        # For the x axis
        dist_x_left = abs(x[0] - self.left_penalty.x1)
        dist_x_right = abs(x[0] - self.right_penalty.x1)
        
        x_distance = min(dist_x_left, dist_x_right) + offset
        
        # For the y axis
        dist_y_left = abs(x[1] - self.left_field_left_penalty.y1)
        dist_y_right = abs(x[1] - self.left_field_right_penalty.y1)

        y_distance = min(dist_y_left, dist_y_right) + offset

        if x_distance < y_distance:
            x_distance = 0
        else:
            y_distance = 0

        return np.array([x[0] + (-1 * np.sign(x[0]) * x_distance + offset), x[1] + (-1 * np.sign(x[1]) * y_distance)])