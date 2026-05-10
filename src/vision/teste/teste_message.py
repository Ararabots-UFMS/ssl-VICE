import pytest
import numpy as np
from unittest.mock import MagicMock
from vision.world_message import wrap_message
from system_interfaces.msg import VisionMessage


def make_obj(x, y, is_ball=False, is_blue=False):
    obj = MagicMock()
    obj.id.is_ball = is_ball
    obj.id.is_blue = is_blue
    obj.id.id = 0
    obj.KF.x = np.array([[x], [y], [0.0], [0.0]])
    obj.orientation_KF.x = np.array([[0.0], [0.0]])
    return obj


def test_invalid_values_are_discarded():
    # Simula objetos com valores absurdos
    objects = {
        "robot1": type("Obj", (), {"position_x": float("nan"), "position_y": 100})(),
        "ball1": type("Obj", (), {"position_x": 999999, "position_y": 999999})(),
    }
    msg = wrap_message(objects)
    assert len(msg.balls) == 0

def test_valid_values_are_added():
    objects = {
        "robot1": type("Obj", (), {"position_x": 100, "position_y": 200, "id": type("ID", (), {"is_ball": False})()})(),
        "ball1": type("Obj", (), {"position_x": 50, "position_y": 60, "id": type("ID", (), {"is_ball": True})()})(),
    }
    msg = wrap_message(objects)
    assert len(msg.yellow_robots) == 1
    assert len(msg.balls) == 1
