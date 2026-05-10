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
        "robot1": make_obj(float("nan"), 100.0, is_ball=False),
        "ball1": make_obj(999999.0, 999999.0, is_ball=True),
    }
    msg = wrap_message(objects)
    # Nenhum objeto inválido deve ser adicionado
    assert len(msg.yellow_robots) == 0
    assert len(msg.balls) == 0

def test_valid_values_are_added():
    objects = {
        "robot1": make_obj(100.0, 200.0, is_ball=False, is_blue=False),
        "ball1": make_obj(50.0, 60.0, is_ball=True),
    }
    msg = wrap_message(objects)
    assert len(msg.yellow_robots) == 1
    assert len(msg.balls) == 1
