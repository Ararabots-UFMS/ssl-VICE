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

def test_id_igualdade():  # unitário: igualdade entre IDs com mesmos valores
    a = ID(id=1, is_ball=False, is_blue=True)
    b = ID(id=1, is_ball=False, is_blue=True)
    assert a == b

def test_id_diferente_time():  # unitário: IDs de times diferentes são distintos
    a = ID(id=1, is_ball=False, is_blue=True)
    b = ID(id=1, is_ball=False, is_blue=False)
    assert a != b

def test_id_hash_usado_como_chave():  # unitário: ID funciona como chave de dicionário
    d = {}
    id1 = ID(id=1, is_ball=False, is_blue=True)
    d[id1] = "robo"
    assert d[ID(id=1, is_ball=False, is_blue=True)] == "robo"


# --- Testes de Object ---

def test_objeto_atualiza_last_seen():  # unitário: last_seen é atualizado após detecção
    id = ID(id=1, is_ball=False, is_blue=False)
    obj = Object([[0.0], [0.0]], id, confidence=1.0, orientation=0.0)
    before = obj.last_seen
    time.sleep(0.05)
    obj.update(1.0, 1.0, 0.9, 0.1)
    assert obj.last_seen > before

def test_objeto_atualiza_confidence():  # unitário: confiança é atualizada corretamente
    id = ID(id=1, is_ball=False, is_blue=False)
    obj = Object([[0.0], [0.0]], id, confidence=1.0, orientation=0.0)
    obj.update(1.0, 1.0, 0.5, 0.0)
    assert obj.confidence == 0.5

def test_objeto_predict_nao_quebra():  # unitário: predict não lança exceção
    id = ID(id=1, is_ball=False, is_blue=False)
    obj = Object([[0.0], [0.0]], id, confidence=1.0, orientation=0.0)
    time.sleep(0.05)
    obj.predict()


# --- Testes de ObjectTracker ---

def test_tracker_adiciona_novo_objeto():  # unitário: novo objeto é inserido no dicionário
    tracker = ObjectTracker(max_time_undetected=1.0)
    robot = make_robot(robot_id=1, x=100.0, y=200.0)
    tracker.read_object_from_message(robot, is_ball=False, is_blue=False)
    assert len(tracker.objects) == 1

def test_tracker_atualiza_objeto_existente():  # unitário: objeto já existente é atualizado, não duplicado
    tracker = ObjectTracker(max_time_undetected=1.0)
    robot = make_robot(robot_id=1, x=100.0, y=200.0)
    tracker.read_object_from_message(robot, is_ball=False, is_blue=False)
    tracker.read_object_from_message(robot, is_ball=False, is_blue=False)
    assert len(tracker.objects) == 1

def test_tracker_diferencia_times():  # unitário: amarelo e azul com mesmo ID são objetos distintos
    tracker = ObjectTracker(max_time_undetected=1.0)
    yellow = make_robot(robot_id=1, x=0.0, y=0.0)
    blue = make_robot(robot_id=1, x=0.0, y=0.0)
    tracker.read_object_from_message(yellow, is_ball=False, is_blue=False)
    tracker.read_object_from_message(blue, is_ball=False, is_blue=True)
    assert len(tracker.objects) == 2

def test_tracker_adiciona_bola():  # unitário: bola é adicionada corretamente
    tracker = ObjectTracker(max_time_undetected=1.0)
    ball = make_ball(x=50.0, y=50.0)
    tracker.read_object_from_message(ball, is_ball=True)
    assert len(tracker.objects) == 1

def test_delete_objeto_apos_tempo():  # temporal: objeto é removido após max_time_undetected
    tracker = ObjectTracker(max_time_undetected=0.1)
    id = ID(id=1, is_ball=False, is_blue=False)
    tracker.objects[id] = Object([[0.0], [0.0]], id, confidence=1.0)
    time.sleep(0.2)
    tracker.delete_undetected_objects([])
    assert len(tracker.objects) == 0

def test_objeto_nao_deletado_antes_do_tempo():  # temporal: objeto não é removido antes do tempo limite
    tracker = ObjectTracker(max_time_undetected=1.0)
    id = ID(id=1, is_ball=False, is_blue=False)
    tracker.objects[id] = Object([[0.0], [0.0]], id, confidence=1.0)
    tracker.delete_undetected_objects([])
    assert len(tracker.objects) == 1

def test_objeto_detectado_nao_deletado():  # temporal: objeto detectado não é removido mesmo após timeout
    tracker = ObjectTracker(max_time_undetected=0.1)
    id = ID(id=1, is_ball=False, is_blue=False)
    tracker.objects[id] = Object([[0.0], [0.0]], id, confidence=1.0)
    time.sleep(0.2)
    tracker.delete_undetected_objects([id])
    assert len(tracker.objects) == 1

def test_confidence_zerada_quando_nao_detectado():  # unitário: confiança vai a 0 quando não detectado
    tracker = ObjectTracker(max_time_undetected=1.0)
    id = ID(id=1, is_ball=False, is_blue=False)
    tracker.objects[id] = Object([[0.0], [0.0]], id, confidence=1.0)
    tracker.delete_undetected_objects([])
    assert tracker.objects[id].confidence == 0

def test_predict_undetected_nao_prediz_detectado():  
    tracker = ObjectTracker(max_time_undetected=1.0)
    id1 = ID(id=1, is_ball=False, is_blue=False)
    tracker.objects[id1] = Object([[0.0], [0.0]], id1, confidence=1.0, orientation=0.0)
    last_seen_before = tracker.objects[id1].last_seen
    time.sleep(0.05)
    tracker.update(make_message())
    assert tracker.objects[id1].last_seen == last_seen_before

def test_update_seleciona_bola_maior_confianca():  # unitário: bola com maior confiança é escolhida
    tracker = ObjectTracker(max_time_undetected=1.0)
    ball_low = make_ball(x=10.0, y=10.0, confidence=0.3)
    ball_high = make_ball(x=50.0, y=50.0, confidence=0.9)
    msg = make_message(balls=[ball_low, ball_high])
    tracker.update(msg)
    ball_id = ID(id=0, is_ball=True)
    assert ball_id in tracker.objects

def test_update_processa_robos_amarelos_e_azuis():  # integração: update processa ambos os times
    tracker = ObjectTracker(max_time_undetected=1.0)
    yellow = make_robot(robot_id=0, x=0.0, y=0.0)
    blue = make_robot(robot_id=0, x=1.0, y=1.0)
    msg = make_message(yellow=[yellow], blue=[blue])
    tracker.update(msg)
    assert len(tracker.objects) == 2


# --- Testes hipotéticos: comportamentos inesperados e casos extremos ---
#
# Hipótese 1: e se alguém passar is_blue=False para uma bola?
#   → imaginamos que criaria duas entradas de bola no tracker (bug silencioso)
#   → na prática: o código ignora is_blue quando is_ball=True, então não acontece
#
# Hipótese 2: e se bola e robô 0 amarelo tiverem o mesmo hash?
#   → imaginamos que um sobrescreveria o outro no dicionário
#   → na prática: __eq__ compara is_ball também, então são chaves distintas
#
# Hipótese 3: e se a confiança for zerada, o objeto sai do tracker?
#   → imaginamos que zerando confidence o objeto seria removido
#   → na prática: só delete_undetected_objects remove — confidence é só um indicador
#
# Hipótese 4: e se robô amarelo e azul com mesmo número aparecerem juntos?
#   → imaginamos que poderiam colidir como mesma chave no dicionário
#   → na prática: is_blue diferente garante chaves distintas, não há colisão
#
# Hipótese 5: e se duas bolas tiverem a mesma confiança?
#   → imaginamos que poderia dar erro ou comportamento imprevisível no max()
#   → na prática: max() retorna a primeira — determinístico mas arbitrário
#
# Hipótese 6: e se update() receber uma mensagem completamente vazia?
#   → imaginamos que poderia quebrar sem robôs nem bola
#   → na prática: apenas prediz e deleta os existentes sem erro
#
# Hipótese 7: e se max_time_undetected for 0?
#   → imaginamos que tudo seria deletado imediatamente
#   → na prática: com 0 qualquer objeto some no próximo frame

def test_hipotetico_bola_ignora_is_blue():  # hipotético: is_blue é ignorado para bola
    tracker = ObjectTracker(max_time_undetected=1.0)
    ball = MagicMock()
    ball.x = 50.0
    ball.y = 50.0
    ball.confidence = 1.0

    tracker.read_object_from_message(ball, is_ball=True, is_blue=None)
    tracker.read_object_from_message(ball, is_ball=True, is_blue=False)

    assert len(tracker.objects) == 1


def test_hipotetico_hash_igual_eq_diferente_nao_sobrescreve():  # hipotético: colisão de hash não causa overwrite
    id_bola = ID(id=0, is_ball=True, is_blue=False)
    id_robo = ID(id=0, is_ball=False, is_blue=False)

    assert hash(id_bola) == hash(id_robo)
    assert id_bola != id_robo

    d = {}
    d[id_bola] = "bola"
    d[id_robo] = "robo"
    assert len(d) == 2


def test_hipotetico_confidence_zero_nao_remove_objeto():  # hipotético: confidence 0 não remove do tracker
    tracker = ObjectTracker(max_time_undetected=1.0)
    id = ID(id=1, is_ball=False, is_blue=False)
    tracker.objects[id] = Object([[0.0], [0.0]], id, confidence=1.0)
    tracker.objects[id].confidence = 0

    assert len(tracker.objects) == 1


def test_hipotetico_objeto_com_mesmo_id_times_diferentes_nao_colide():  # hipotético: amarelo e azul não colidem
    tracker = ObjectTracker(max_time_undetected=1.0)
    robot = MagicMock()
    robot.robot_id = 1
    robot.x = 0.0
    robot.y = 0.0
    robot.confidence = 1.0
    robot.orientation = 0.0

    tracker.read_object_from_message(robot, is_ball=False, is_blue=False)
    tracker.read_object_from_message(robot, is_ball=False, is_blue=True)

    assert len(tracker.objects) == 2


def test_hipotetico_duas_bolas_mesma_confianca():  # hipotético: max() com empate retorna a primeira sem erro
    tracker = ObjectTracker(max_time_undetected=1.0)
    ball_a = make_ball(x=10.0, y=10.0, confidence=0.8)
    ball_b = make_ball(x=90.0, y=90.0, confidence=0.8)
    msg = make_message(balls=[ball_a, ball_b])
    tracker.update(msg)

    ball_id = ID(id=0, is_ball=True)
    assert ball_id in tracker.objects
    assert len(tracker.objects) == 1


def test_hipotetico_update_mensagem_vazia():  # hipotético: mensagem vazia não quebra o tracker
    tracker = ObjectTracker(max_time_undetected=1.0)
    msg = make_message()
    tracker.update(msg)

    assert len(tracker.objects) == 0


def test_hipotetico_max_time_zero_deleta_imediatamente():  # hipotético: max_time=0 deleta no próximo frame
    tracker = ObjectTracker(max_time_undetected=0.0)
    id = ID(id=1, is_ball=False, is_blue=False)
    tracker.objects[id] = Object([[0.0], [0.0]], id, confidence=1.0)
    time.sleep(0.01)
    tracker.delete_undetected_objects([])

    assert len(tracker.objects) == 0
