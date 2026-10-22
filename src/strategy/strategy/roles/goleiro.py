"""GOLEIRO: the robot with GOALKEEPER_ID, run by tatics/goalkeeper.py."""

from strategy.analysis.jogo import AnaliseJogo
from strategy.context import GOALKEEPER_ID
from strategy.tatics.goalkeeper import Goalkeeper

PAPEL_GOLEIRO = "goleiro"


class Goleiro:
    """Not a Papel: the keeper has its own kick and obstacle rules."""

    nome = PAPEL_GOLEIRO

    def comando(self, a: AnaliseJogo):
        mundo = a.mundo
        gk = Goalkeeper(mundo.aliados[GOALKEEPER_ID], mundo.bola, mundo.on_positive_half,
                        mundo.aliados, mundo.inimigos)
        return gk.execute(mundo.nosso_gol, mundo.bola)
