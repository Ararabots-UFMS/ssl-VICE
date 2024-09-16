from .interfaces import StaticObstacle

class BoundaryObstacles(StaticObstacle):
    def __init__(self):
        pass
    
    def is_colission(self, x: np.matrix, ignore: bool = False):
        pass

    def closest_outside_point(self, x: np.matrix):
        pass

class FieldObstacles(StaticObstacle):
    def __init__(self):
        pass

    def is_colission(self, x: np.matrix, ignore: bool = False):
        pass

    def closest_outside_point(self, x: np.matrix):
        pass