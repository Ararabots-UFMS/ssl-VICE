"""Open play (FORCE_START / NORMAL_START): analyse once, assign roles, ask each role.

    analysis/jogo.analisar      situation, roles, kick target    (once per tick)
    roles/<papel>.comando       one Skill per robot              (per robot)

To add a role: write roles/<nome>.py with a Papel subclass, give it a name in
roles/atribuicao.py, hand it out in distribuir_papeis and register it in
JogoCorrido.papeis below. See docs/estrategia-regras.md.
"""

from dataclasses import dataclass, field
from typing import Dict, List

from strategy.analysis.jogo import AnaliseJogo, MemoriaAnalise, analisar
from strategy.context import GOALKEEPER_ID, TickContext
from strategy.roles.apoio import Apoio
from strategy.roles.atribuicao import PAPEL_APOIO, PAPEL_COBERTURA, PAPEL_PORTADOR
from strategy.roles.base import Papel
from strategy.roles.cobertura import Cobertura
from strategy.roles.goleiro import Goleiro
from strategy.roles.portador import Portador
from strategy.skills.skills import Skill

# A field player without a role this tick covers.
PAPEL_PADRAO = PAPEL_COBERTURA


@dataclass
class MemoriaJogo:
    analise: MemoriaAnalise = field(default_factory=MemoriaAnalise)
    # Kick arming latch per robot id; see skills/chute.armar_chute.
    travas_chute: Dict[int, object] = field(default_factory=dict)


class JogoCorrido:
    """Lives as long as the play, so its memory and its roles' memory survive ticks."""

    def __init__(self):
        self.memoria = MemoriaJogo()
        self.papeis: Dict[str, Papel] = {
            PAPEL_PORTADOR: Portador(),
            PAPEL_APOIO: Apoio(),
            PAPEL_COBERTURA: Cobertura(),
        }
        self.goleiro = Goleiro()

    def comandos(self, context: TickContext) -> List[Skill]:
        a = analisar(context, self.memoria.analise)
        comandos = []
        if GOALKEEPER_ID in a.mundo.aliados:
            comandos.append(self.goleiro.comando(a))
        comandos.extend(self._comandos_de_linha(a))
        return comandos

    def _comandos_de_linha(self, a: AnaliseJogo) -> List[Skill]:
        ordem = {nome: 0 for nome in self.papeis}
        comandos = []
        for rid in a.mundo.linha:
            nome = a.papeis.get(rid, PAPEL_PADRAO)
            comandos.append(self.papeis[nome].comando(a, rid, ordem[nome],
                                                      self.memoria.travas_chute))
            ordem[nome] += 1
        return comandos
