import pytest

class Vector2D:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class MotionPrimitive:
    def __init__(self, start, end, duration):
        self.start = start
        self.end = end
        self.duration = duration

class MotionPath:
    def __init__(self, primitives=None):
        self.primitives = primitives or []

    def duration(self):
        return sum(p.duration for p in self.primitives)

    def truncate(self, t):
        if t <= 0:
            self.primitives = []
            return
        total = 0
        new_primitives = []
        for p in self.primitives:
            if total + p.duration < t:
                new_primitives.append(p)
                total += p.duration
            else:
                if t - total > 0:
                    frac = (t - total) / p.duration
                    new_end = Vector2D(
                        p.start.x + frac * (p.end.x - p.start.x),
                        p.start.y + frac * (p.end.y - p.start.y)
                    )
                    new_primitives.append(MotionPrimitive(p.start, new_end, t - total))
                break
        self.primitives = new_primitives


def test_motion_path_creation():
    p1 = MotionPrimitive(Vector2D(0,0), Vector2D(1,1), 2)
    p2 = MotionPrimitive(Vector2D(1,1), Vector2D(2,2), 3)
    path = MotionPath([p1, p2])
    assert len(path.primitives) == 2
    assert path.duration() == 5

def test_truncate_zero():
    p1 = MotionPrimitive(Vector2D(0,0), Vector2D(1,1), 2)
    path = MotionPath([p1])
    path.truncate(0)
    assert len(path.primitives) == 0

def test_truncate_full():
    p1 = MotionPrimitive(Vector2D(0,0), Vector2D(1,1), 2)
    p2 = MotionPrimitive(Vector2D(1,1), Vector2D(2,2), 3)
    path = MotionPath([p1, p2])
    path.truncate(5)
    assert len(path.primitives) == 2
    assert path.duration() == 5

def test_truncate_partial():
    p1 = MotionPrimitive(Vector2D(0,0), Vector2D(1,1), 2)
    p2 = MotionPrimitive(Vector2D(1,1), Vector2D(2,2), 3)
    path = MotionPath([p1, p2])
    path.truncate(3)
    assert len(path.primitives) == 2
    assert path.primitives[1].duration == 1
    assert path.primitives[1].end.x == 1 + (1/3)*(2-1)
    assert path.primitives[1].end.y == 1 + (1/3)*(2-1)

def test_truncate_on_primitive_boundary():
    p1 = MotionPrimitive(Vector2D(0,0), Vector2D(1,1), 2)
    p2 = MotionPrimitive(Vector2D(1,1), Vector2D(2,2), 3)
    path = MotionPath([p1, p2])
    path.truncate(2)
    assert len(path.primitives) == 1
    assert path.primitives[0].duration == 2
    assert path.primitives[0].end.x == 1
    assert path.primitives[0].end.y == 1