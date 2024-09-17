from .interfaces import StaticObstacle

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
            x_distance  = x[0] - self.half_length

        if x[1] < -1 * self.half_width or x[1] > self.half_width:
            y_distance  = x[1] - self.half_width

        # Huge mamaco, there may be a better way...
        return np.array([x[0] + (-1 * np.sign(x[0]) * x_distance), x[1] + (-1 * np.sign(x[1]) * y_distance)])

class WallObstacles(StaticObstacle):
    def __init__(self, geometry: VisionGeometry):
        self.half_width = geometry.field_width / 2
        self.half_length = geometry.field_length / 2   
        self.boundary_width = geometry.boundary_width

    def is_colission(self, x: np.matrix, ignore: bool = False, padding: float = 90):
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

        # Huge mamaco, there may be a better way...
        return np.array([x[0] + (-1 * np.sign(x[0]) * x_distance), x[1] + (-1 * np.sign(x[1]) * y_distance)])


class FieldObstacles(StaticObstacle):
    def __init__(self):
        pass

    def is_colission(self, x: np.matrix, ignore: bool = False, padding: float = 90):
        pass

    def closest_outside_point(self, x: np.matrix):
        pass