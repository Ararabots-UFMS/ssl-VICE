import pytest
from vision.world_message import wrap_message
from system_interfaces.msg import VisionMessage

def test_invalid_values_are_discarded():
    # Simula objetos com valores absurdos
    objects = {
        "robot1": type("Obj", (), {"position_x": float("nan"), "position_y": 100})(),
        "ball1": type("Obj", (), {"position_x": 999999, "position_y": 999999})(),
    }
    msg = wrap_message(objects)
    # Nenhum objeto inválido deve ser adicionado
    assert len(msg.yellow_robots) == 0
    assert len(msg.balls) == 0

def test_valid_values_are_added():
    objects = {
        "robot1": type("Obj", (), {"position_x": 100, "position_y": 200, "id": type("ID", (), {"is_ball": False})()})(),
        "ball1": type("Obj", (), {"position_x": 50, "position_y": 60, "id": type("ID", (), {"is_ball": True})()})(),
    }
    msg = wrap_message(objects)
    assert len(msg.yellow_robots) == 1
    assert len(msg.balls) == 1
