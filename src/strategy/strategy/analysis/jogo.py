"""The full open-play analysis of one tick, built once and handed to every role."""

from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

from strategy import diagnostico
from strategy.analysis.alvo_chute import AlvoChute, MemoriaMira, alvo_do_chute
from strategy.analysis.mundo import Mundo, analisar_mundo
from strategy.context import TickContext
from strategy.roles.atribuicao import MemoriaPapeis, distribuir_papeis, eleger_buscador
from strategy.skills import experimento


@dataclass
class MemoriaAnalise:
    """What the analysis keeps between ticks. Owned by the play, never global."""

    papeis: MemoriaPapeis = field(default_factory=MemoriaPapeis)
    mira: MemoriaMira = field(default_factory=MemoriaMira)


@dataclass(frozen=True)
class AnaliseJogo:
    mundo: Mundo
    papeis: Dict[int, str]
    alvo: AlvoChute
    # With no carrier assigned, the defender that goes for the ball.
    buscador: Optional[int] = None

    # Shortcuts for the fields every role reads.
    @property
    def bola(self):
        return self.mundo.bola

    @property
    def alvo_chute(self) -> Optional[Tuple[float, float]]:
        return self.alvo.ponto

    @property
    def tipo_alvo(self) -> Optional[str]:
        return self.alvo.tipo


def analisar(context: TickContext, memoria: MemoriaAnalise) -> AnaliseJogo:
    mundo = analisar_mundo(context)
    papeis = distribuir_papeis(mundo, memoria.papeis)
    diagnostico.publicar_papeis(papeis)
    if diagnostico.ativo():
        diagnostico.log("[JG] chaves=%s" % experimento.estado())
        diagnostico.log("[JG] situacao=%s papeis=%s" % (mundo.situacao, papeis))
        diagnostico.log("[JG] lado=%s gol_ataque=%.0f nosso_gol=%.0f bola=%.0f,%.0f"
                        % (mundo.on_positive_half, mundo.gol_ataque.x, mundo.nosso_gol.x,
                           mundo.bola.position_x, mundo.bola.position_y))
    alvo = alvo_do_chute(mundo, papeis, memoria.mira)
    return AnaliseJogo(mundo=mundo, papeis=papeis, alvo=alvo,
                       buscador=eleger_buscador(mundo, papeis))
