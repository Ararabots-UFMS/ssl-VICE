from movement.obstacles.interfaces import StaticObstacle
from system_interfaces.msg import VisionGeometry

from typing import List, Tuple

from math import copysign

# This needs to change to use the FieldLineSegment instead of the distances
class BoundaryObstacles(StaticObstacle):
    def __init__(self, geometry: VisionGeometry):
        for line in geometry.field_lines:
                
                if line.name == 'TopTouchLine':
                    self.top_line = line
    
                elif line.name == 'BottomTouchLine':
                    self.bottom_line = line
    
                elif line.name == 'LeftGoalLine':
                    self.left_goal_line = line
    
                elif line.name == 'RightGoalLine':
                    self.right_goal_line = line
    
    def is_colission(self, point: Tuple[float, float], ignore: bool = False, padding: float = 90) -> bool:
        '''
        Check if the point is inside the field boundaries

        Parameters:
        point (Tuple[float, float]): The point to check
        ignore (bool): If the point should be ignored
        padding (float): The padding to consider the point inside the field

        Returns:
        bool: If the point is inside the field boundaries
        ''' 
        
        if ignore:
            return False
        elif point[0] < self.left_goal_line.x1 + padding or point[0] > self.right_goal_line.x1 - padding:
            return True
        elif point[1] < self.bottom_line.y1 + padding or point[1] > self.top_line.y1 - padding:
            return True
        else:
            return False

    def closest_outside_point(self, point: Tuple[float, float], offset: float = 90) -> Tuple[float, float]:
        
        '''
        Return the closest point outside the field boundaries on the x and y axis

        Parameters:
        point (Tuple[float, float]): The point to check
        offset (float): The offset to consider the point outside the field

        Returns:
        Tuple[float, float]: The closest point outside the field boundaries
        '''
        
        x_distance = 0
        y_distance = 0
        
        if point[0] < self.left_goal_line.x1:
            x_distance = point[0] - self.left_goal_line.x1 + offset
        
        elif point[0] > self.right_goal_line.x1:
            x_distance = point[0] - self.right_goal_line.x1 - offset

        if point[1] < self.bottom_line.y1:
            y_distance = point[1] - self.bottom_line.y1 + offset
            
        elif point[1] > self.top_line.y1:
            y_distance = point[1] - self.top_line.y1 - offset

        if x_distance < y_distance:
            x_distance = 0
        else:
            y_distance = 0

        # Huge mamaco, there may be a better way...
        return [point[0] - copysign(x_distance, point[0]), point[1] - copysign(y_distance, point[1])]

class WallObstacles(StaticObstacle):
    def __init__(self, geometry: VisionGeometry):
        for line in geometry.field_lines:
            if line.name == 'TopTouchLine':
                self.top_line = line

            elif line.name == 'BottomTouchLine':
                self.bottom_line = line

            elif line.name == 'LeftGoalLine':
                self.left_goal = line

            elif line.name == 'RightGoalLine':
                self.right_goal = line
            elif line.name == 'boundary_width':
                self.boundary_width = line


    def is_colission(self, point: Tuple[float, float], ignore: bool = False, padding: float = 180) -> bool:
        '''
        Check if the point is inside the field boundaries

        Parameters:
        point (Tuple[float, float]): The point to check
        ignore (bool): If the point should be ignored
        padding (float): The padding to consider the point inside the field

        Returns:
        bool: If the point is inside the field boundaries
        '''
        
        if ignore:
            return False
        elif point[0] < (self.left_goal.x1 - self.boundary_width + padding) or point[0] > (self.right_goal.x1 + self.boundary_width - padding):
            return True
        elif point[1] < (self.bottom_line.y1 - self.boundary_width + padding) or point[1] > (self.top_line.y1 + self.boundary_width - padding):
            return True
        else:
            return False

    def closest_outside_point(self, point: Tuple[float, float], offset: float) -> Tuple[float, float]:
        '''
        Return the closest point outside the field boundaries on the x and y axis

        Parameters:
        point (Tuple[float, float]): The point to check
        offset (float): The offset to consider the point outside the field

        Returns:
        Tuple[float, float]: The closest point outside the field boundaries
        '''

        x_distance = 0
        y_distance = 0

        if point[0] < self.left_goal.x1 - self.boundary_width:
            x_distance = point[0] - self.left_goal.x1 + self.boundary_width + offset
        
        elif point[0] > self.right_goal.x1 + self.boundary_width:
            x_distance = point[0] - self.right_goal.x1 - self.boundary_width - offset

        if point[1] < self.bottom_line.y1 - self.boundary_width:
            y_distance = point[1] - self.bottom_line.y1 + self.boundary_width + offset

        elif point[1] > self.top_line.y1 + self.boundary_width:
            y_distance = point[1] - self.top_line.y1 - self.boundary_width - offset

        if x_distance < y_distance:
            x_distance = 0
        else:
            y_distance = 0

        # # TODO Review the offset parameters...
        return [point[0] - copysign(x_distance, point[0]) + offset, point[1] - copysign(y_distance, point[1])]

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

    def is_colission(self, point: Tuple[float, float], ignore: bool = False, padding: float = 90) -> bool:
        '''
        Check if the point is inside the field boundaries

        Parameters:
        point (Tuple[float, float]): The point to check

        Returns:
        bool: If the point is inside the field boundaries
        '''
        
        
        if ignore:
            return False
        # For the left field side
        # -4self.boundary_width 1000
        if point[0] > self.left_field_left_penalty.x1 and point[0] < self.left_field_left_penalty.x2 + padding:
            if point[1] < self.left_field_left_penalty.y1 + padding and point[1] > self.left_field_right_penalty.y1 - padding:
                return True

        # For the right field side
        if point[0] < self.right_field_left_penalty.x1 and point[0] > self.right_field_left_penalty.x2 - padding:
            if point[1] > self.right_field_left_penalty.y1 - padding and point[1] < self.right_field_right_penalty.y1 + padding:
                return True

        return False

    def closest_outside_point(self, point: Tuple[float, float], offset: float = 90) -> Tuple[float, float]:
        '''
        Return the closest point outside the field boundaries on the x and y axis
        
        Parameters:
        point (Tuple[float, float]): The point to check
        offset (float): The offset to consider the point outside the field

        Returns:
        Tuple[float, float]: The closest point outside the field boundaries
        '''

        x_distance = 0
        y_distance = 0

        # For the x axis
        dist_x_left = abs(point[0] - self.left_penalty.x1)
        dist_x_right = abs(point[0] - self.right_penalty.x1)
        
        x_distance = min(dist_x_left, dist_x_right) + offset
        
        # For the y axis
        dist_y_left = abs(point[1] - self.left_field_left_penalty.y1)
        dist_y_right = abs(point[1] - self.left_field_right_penalty.y1)

        y_distance = min(dist_y_left, dist_y_right) + offset

        if x_distance < y_distance:
            x_distance = 0
        else:
            y_distance = 0
 
        # TODO Review offset parameters too...
        return [point[0] - copysign(x_distance, point[0]) + offset, point[1] - copysign(y_distance, point[1])]