from movement.obstacles.interfaces import StaticObstacle
from system_interfaces.msg import VisionGeometry

from typing import Tuple

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
                
    
    def is_colission(self, point: Tuple[float, float], padding: float = 90) -> bool:
        '''
        Check if the point is inside the field boundaries

        Parameters:
        point (Tuple[float, float]): The point to check
        ignore (bool): If the point should be ignored
        padding (float): The padding to consider the point inside the field

        Returns:
        bool: If the point is inside the field boundaries
        ''' 

        if point[0] < self.left_goal_line.x1 + padding or point[0] > self.right_goal_line.x1 - padding:
            return True

        if point[1] < self.bottom_line.y1 + padding or point[1] > self.top_line.y1 - padding:
            return True

        return False

    def closest_outside_point(self, point: Tuple[float, float], offset: float = 90, padding: float = 90) -> Tuple[float, float]:
        
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
        
        if point[0] < self.left_goal_line.x1 + padding:
            x_distance = abs(point[0] - self.left_goal_line.x1) + padding  + offset

        elif point[0] > self.right_goal_line.x1 - padding:
            x_distance = abs(point[0] - self.right_goal_line.x1) + padding + offset

        if point[1] < self.bottom_line.y1 + padding:
            y_distance = abs(point[1] - self.bottom_line.y1) + padding + offset
            
        elif point[1] > self.top_line.y1 - padding:
            y_distance = abs(point[1] - self.top_line.y1) + padding + offset

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
        
        self.boundary_width = geometry.boundary_width


    def is_colission(self, point: Tuple[float, float], padding: float = 90) -> bool:
        '''
        Check if the point is inside the field boundaries

        Parameters:
        point (Tuple[float, float]): The point to check
        ignore (bool): If the point should be ignored
        padding (float): The padding to consider the point inside the field

        Returns:
        bool: If the point is inside the field boundaries
        '''
        
        
        if point[0] < (self.left_goal.x1 - self.boundary_width + padding) or point[0] > (self.right_goal.x1 + self.boundary_width - padding):
            return True
        elif point[1] < (self.bottom_line.y1 - self.boundary_width + padding) or point[1] > (self.top_line.y1 + self.boundary_width - padding):
            return True
        
        return False

    def closest_outside_point(self, point: Tuple[float, float], offset: float = 20, padding: float = 90) -> Tuple[float, float]:
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

        if point[0] < self.left_goal.x1 - self.boundary_width + padding:
            x_distance = abs(point[0] - self.left_goal.x1) + offset + padding
        
        elif point[0] > self.right_goal.x1 + self.boundary_width - padding:
            x_distance = abs(point[0] - self.right_goal.x1) + offset + padding

        if point[1] < self.bottom_line.y1 - self.boundary_width + padding:
            y_distance = abs(point[1] - self.bottom_line.y1) + offset + padding

        elif point[1] > self.top_line.y1 + self.boundary_width - padding:
            y_distance = abs(point[1] - self.top_line.y1) + offset + padding


        return [point[0] - copysign(x_distance, point[0]), point[1] - copysign(y_distance, point[1])]

class PenaltyAreaObstacles(StaticObstacle):
    def __init__(self, geometry: VisionGeometry):
        for line in geometry.field_lines:

            if line.name == 'LeftPenaltyStretch':
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

    def is_colission(self, point: Tuple[float, float], padding: float = 110) -> bool:
        '''
        Check if the point is inside the field boundaries

        Parameters:
        point (Tuple[float, float]): The point to check

        Returns:
        bool: If the point is inside the field boundaries
        '''
        
        
        # For the left field area
        if point[0] > self.left_field_left_penalty.x1 - padding and point[0] < self.left_field_left_penalty.x2 + padding:
            if point[1] < self.left_field_left_penalty.y1 + padding and point[1] > self.left_field_right_penalty.y1 - padding:
                return True

        # For the right field side
        if point[0] < self.right_field_left_penalty.x1 + padding  and point[0] > self.right_field_left_penalty.x2 - padding:
            if point[1] > self.right_field_left_penalty.y1 - padding and point[1] < self.right_field_right_penalty.y1 + padding:
                return True

        return False

    def closest_outside_point(self, point: Tuple[float, float], offset: float = 10, padding: float = 90) -> Tuple[float, float]:
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

        if point[0] > self.left_field_left_penalty.x1 - padding and point[0] < self.left_field_left_penalty.x2 + padding:
            if point[1] < self.left_field_left_penalty.y1 + padding and point[1] > self.left_field_right_penalty.y1 - padding:
                
                x_distance = abs(point[0] - self.left_penalty.x1) + padding + offset

                y_distance = min(abs(point[1] - self.left_field_left_penalty.y1), abs(point[1] - self.left_field_right_penalty.y1)) + padding + offset
                
        
        if point[0] < self.right_field_left_penalty.x1 + padding  and point[0] > self.right_field_left_penalty.x2 - padding:
            if point[1] > self.right_field_left_penalty.y1 - padding and point[1] < self.right_field_right_penalty.y1 + padding:
                
                x_distance = abs(point[0] - self.right_penalty.x1) + padding + offset

                y_distance = min(abs(point[1] - self.right_field_left_penalty.y1), abs(point[1] - self.right_field_right_penalty.y1)) + padding + offset

        if x_distance > y_distance:
            x_distance = 0
        else:
            y_distance = 0
 
        return [point[0] - copysign(x_distance, point[0]), point[1] + copysign(y_distance, point[1])]