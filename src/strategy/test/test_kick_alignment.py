from math import atan2
from types import SimpleNamespace as S

import pytest

from strategy.skills import chute
from strategy.tatics.goalkeeper import Goalkeeper


@pytest.mark.parametrize("alvo,tipo,forca", [
    ((4500.0, 0.0), "gol", chute.FORCA_CHUTE),
    ((1600.0, 400.0), "passe", chute.FORCA_PASSE),
])
def test_chute_espera_alinhamento_medido_com_alvo(alvo, tipo, forca):
    robo = S(position_x=0.0, position_y=0.0, orientation=0.6)
    bola = S(position_x=120.0, position_y=0.0)
    travas = {}

    ang, armado, potencia = chute.chutar_em(
        robo, bola, alvo, tipo, 1, travas, 1)
    assert ang == pytest.approx(atan2(alvo[1], alvo[0] - bola.position_x))
    assert not armado and potencia == 0.0

    robo.orientation = ang
    _, armado, potencia = chute.chutar_em(
        robo, bola, alvo, tipo, 1, travas, 1)
    assert armado and potencia == forca

    # A trava de proximidade nao pode manter o chute se a mira se perder.
    robo.orientation = 0.6
    _, armado, potencia = chute.chutar_em(
        robo, bola, alvo, tipo, 1, travas, 1)
    assert not armado and potencia == 0.0
    assert not travas[1]


def test_goleiro_so_passa_depois_de_apontar_para_companheiro():
    goleiro = S(position_x=-4120.0, position_y=0.0, orientation=0.6)
    bola = S(position_x=-4000.0, position_y=0.0)
    receptor = S(position_x=-2300.0, position_y=0.0)
    tatica = Goalkeeper(goleiro, bola, False, {0: goleiro, 1: receptor})
    nossa_meta = S(x=-4500.0, y=0.0)

    aguardando = tatica.execute(nossa_meta, bola)
    assert aguardando.angle == pytest.approx(0.0)
    assert aguardando.kick == 0.0

    goleiro.orientation = 0.0
    alinhado = tatica.execute(nossa_meta, bola)
    assert alinhado.kick == 2.5


def test_goleiro_sem_receptor_espera_alinhamento_para_saida():
    goleiro = S(position_x=-4120.0, position_y=0.0, orientation=0.6)
    bola = S(position_x=-4000.0, position_y=0.0)
    tatica = Goalkeeper(goleiro, bola, False, {0: goleiro})
    nossa_meta = S(x=-4500.0, y=0.0)

    aguardando = tatica.execute(nossa_meta, bola)
    assert aguardando.kick == 0.0

    goleiro.orientation = aguardando.angle
    alinhado = tatica.execute(nossa_meta, bola)
    assert alinhado.kick == 4.5
