import time
import pytest
from unittest.mock import MagicMock
from vision.tracker import ObjectTracker, Object, ID


def make_robot(robot_id, x, y, confidence=1.0, orientation=0.0):
    robot = MagicMock()
    robot.robot_id = robot_id
    robot.x = x
    robot.y = y
    robot.confidence = confidence
    robot.orientation = orientation
    return robot


def make_ball(x, y, confidence=1.0):
    ball = MagicMock()
    ball.x = x
    ball.y = y
    ball.confidence = confidence
    return ball


def make_message(yellow=None, blue=None, balls=None, t_capture=1.0):
    msg = MagicMock()
    msg.detection.t_capture = t_capture
    msg.detection.robots_yellow = yellow or []
    msg.detection.robots_blue = blue or []
    msg.detection.balls = balls or []
    return msg


# --- Testes de ID ---

def test_id_igualdade():
    a = ID(id=1, is_ball=False, is_blue=True)
    b = ID(id=1, is_ball=False, is_blue=True)
    assert a == b

def test_id_diferente_time():
    a = ID(id=1, is_ball=False, is_blue=True)
    b = ID(id=1, is_ball=False, is_blue=False)
    assert a != b

def test_id_hash_usado_como_chave():
    d = {}
    id1 = ID(id=1, is_ball=False, is_blue=True)
    d[id1] = "robo"
    assert d[ID(id=1, is_ball=False, is_blue=True)] == "robo"


# --- Testes de Object ---

def test_objeto_atualiza_last_seen():
    id = ID(id=1, is_ball=False, is_blue=False)
    obj = Object([[0.0], [0.0]], id, confidence=1.0, orientation=0.0)
    before = obj.last_seen
    time.sleep(0.05)
    obj.update(1.0, 1.0, 0.9, 0.1)
    assert obj.last_seen > before

def test_objeto_atualiza_confidence():
    id = ID(id=1, is_ball=False, is_blue=False)
    obj = Object([[0.0], [0.0]], id, confidence=1.0, orientation=0.0)
    obj.update(1.0, 1.0, 0.5, 0.0)
    assert obj.confidence == 0.5

def test_objeto_predict_nao_quebra():
    id = ID(id=1, is_ball=False, is_blue=False)
    obj = Object([[0.0], [0.0]], id, confidence=1.0, orientation=0.0)
    time.sleep(0.05)
    obj.predict()


# --- Testes de ObjectTracker ---

def test_tracker_adiciona_novo_objeto():
    tracker = ObjectTracker(max_time_undetected=1.0)
    robot = make_robot(robot_id=1, x=100.0, y=200.0)
    tracker.read_object_from_message(robot, is_ball=False, is_blue=False)
    assert len(tracker.objects) == 1

def test_tracker_atualiza_objeto_existente():
    tracker = ObjectTracker(max_time_undetected=1.0)
    robot = make_robot(robot_id=1, x=100.0, y=200.0)
    tracker.read_object_from_message(robot, is_ball=False, is_blue=False)
    tracker.read_object_from_message(robot, is_ball=False, is_blue=False)
    assert len(tracker.objects) == 1

def test_tracker_diferencia_times():
    tracker = ObjectTracker(max_time_undetected=1.0)
    yellow = make_robot(robot_id=1, x=0.0, y=0.0)
    blue = make_robot(robot_id=1, x=0.0, y=0.0)
    tracker.read_object_from_message(yellow, is_ball=False, is_blue=False)
    tracker.read_object_from_message(blue, is_ball=False, is_blue=True)
    assert len(tracker.objects) == 2

def test_tracker_adiciona_bola():
    tracker = ObjectTracker(max_time_undetected=1.0)
    ball = make_ball(x=50.0, y=50.0)
    tracker.read_object_from_message(ball, is_ball=True)
    assert len(tracker.objects) == 1

def test_delete_objeto_apos_tempo():
    tracker = ObjectTracker(max_time_undetected=0.1)
    id = ID(id=1, is_ball=False, is_blue=False)
    tracker.objects[id] = Object([[0.0], [0.0]], id, confidence=1.0)
    time.sleep(0.2)
    tracker.delete_undetected_objects([])
    assert len(tracker.objects) == 0

def test_objeto_nao_deletado_antes_do_tempo():
    tracker = ObjectTracker(max_time_undetected=1.0)
    id = ID(id=1, is_ball=False, is_blue=False)
    tracker.objects[id] = Object([[0.0], [0.0]], id, confidence=1.0)
    tracker.delete_undetected_objects([])
    assert len(tracker.objects) == 1

def test_objeto_detectado_nao_deletado():
    tracker = ObjectTracker(max_time_undetected=0.1)
    id = ID(id=1, is_ball=False, is_blue=False)
    tracker.objects[id] = Object([[0.0], [0.0]], id, confidence=1.0)
    time.sleep(0.2)
    tracker.delete_undetected_objects([id])
    assert len(tracker.objects) == 1

def test_confidence_zerada_quando_nao_detectado():
    tracker = ObjectTracker(max_time_undetected=1.0)
    id = ID(id=1, is_ball=False, is_blue=False)
    tracker.objects[id] = Object([[0.0], [0.0]], id, confidence=1.0)
    tracker.delete_undetected_objects([])
    assert tracker.objects[id].confidence == 0

def test_predict_undetected_nao_prediz_detectado():
    tracker = ObjectTracker(max_time_undetected=1.0)
    id1 = ID(id=1, is_ball=False, is_blue=False)
    id2 = ID(id=2, is_ball=False, is_blue=False)
    tracker.objects[id1] = Object([[0.0], [0.0]], id1, confidence=1.0, orientation=0.0)
    tracker.objects[id2] = Object([[0.0], [0.0]], id2, confidence=1.0, orientation=0.0)
    last_seen_before = tracker.objects[id1].last_seen
    time.sleep(0.05)
    tracker.predict_undetected([id1])
    assert tracker.objects[id1].last_seen == last_seen_before

def test_update_seleciona_bola_maior_confianca():
    tracker = ObjectTracker(max_time_undetected=1.0)
    ball_low = make_ball(x=10.0, y=10.0, confidence=0.3)
    ball_high = make_ball(x=50.0, y=50.0, confidence=0.9)
    msg = make_message(balls=[ball_low, ball_high])
    tracker.update(msg)
    ball_id = ID(id=0, is_ball=True)
    assert ball_id in tracker.objects

def test_update_processa_robos_amarelos_e_azuis():
    tracker = ObjectTracker(max_time_undetected=1.0)
    yellow = make_robot(robot_id=0, x=0.0, y=0.0)
    blue = make_robot(robot_id=0, x=1.0, y=1.0)
    msg = make_message(yellow=[yellow], blue=[blue])
    tracker.update(msg)
    assert len(tracker.objects) == 2
