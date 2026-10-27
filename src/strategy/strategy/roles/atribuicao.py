"""Who plays which role this tick. The only place roles are handed out.

  PORTADOR   has the ball or goes for it. Always exactly one; the closest to it.
  APOIO      helps without the ball: offers a pass or marks. The most advanced left.
  COBERTURA  between the ball and our goal; second to the ball in DELES/DISPUTA.

With fewer field players than roles the priority is PORTADOR -> COBERTURA -> APOIO:
someone must go to the ball, and an open goal line costs more than a lost pass line.
"""

from dataclasses import dataclass
from typing import Dict, Optional

from strategy import diagnostico
from strategy.analysis.mundo import Mundo, dist
from strategy.skills import experimento

PAPEL_PORTADOR = "portador"
PAPEL_APOIO = "apoio"
PAPEL_COBERTURA = "cobertura"

# A role only changes hands for a clear gain. 600 mm still swapped under pressure,
# and each swap throws away the run the robot already made.
VANTAGEM_TROCA = 1200.0
# And never within 2 s of the last change (20 ticks at 10 Hz): when two robots
# cross, the distance difference passes through zero anyway. Vail & Veloso 2003.
CICLOS_MIN_PAPEL = 20


@dataclass
class MemoriaPapeis:
    portador: Optional[int] = None
    portador_ciclos: int = 0
    apoio: Optional[int] = None
    apoio_ciclos: int = 0


def distribuir_papeis(mundo: Mundo, memoria: MemoriaPapeis) -> Dict[int, str]:
    linha = mundo.linha
    if not linha:
        return {}
    aliados, ball = mundo.aliados, mundo.bola

    # Test mode: everyone defends, and the closest defender goes for the ball.
    if experimento.ligado("FORCAR_COBERTURA"):
        return {rid: PAPEL_COBERTURA for rid in linha}

    # Experiment: roles fixed by id, to isolate the election from its hysteresis.
    if experimento.ligado("PAPEIS_FIXOS"):
        fixos = {}
        for i, rid in enumerate(linha):
            fixos[rid] = (PAPEL_PORTADOR if i == 0 else
                          PAPEL_APOIO if i == 1 else PAPEL_COBERTURA)
        return fixos

    def _adiantado(rid):
        return aliados[rid].position_x * mundo.sentido

    # ---- PORTADOR: closest to the ball, with hysteresis
    portador = min(linha, key=lambda rid: dist(aliados[rid], ball))
    ant = memoria.portador
    if ant is not None and ant in linha and ant != portador:
        travado = memoria.portador_ciclos < CICLOS_MIN_PAPEL
        if travado or (dist(aliados[ant], ball) - dist(aliados[portador], ball)
                       < VANTAGEM_TROCA):
            portador = ant
    if ant is not None and ant != portador:
        diagnostico.log("[TR] troca=portador sai=r%s d_sai=%.0f entra=r%s d_entra=%.0f"
                        % (ant, dist(aliados[ant], ball) if ant in aliados else -1,
                           portador, dist(aliados[portador], ball)))
    memoria.portador_ciclos = memoria.portador_ciclos + 1 if portador == ant else 0
    memoria.portador = portador

    sobram = [rid for rid in linha if rid != portador]
    if not sobram:
        return {portador: PAPEL_PORTADOR}
    if len(sobram) == 1:
        return {portador: PAPEL_PORTADOR, sobram[0]: PAPEL_COBERTURA}

    # ---- APOIO: the most advanced of the rest, with hysteresis
    apoio = max(sobram, key=_adiantado)
    ant = memoria.apoio
    if ant is not None and ant in sobram and ant != apoio:
        travado = memoria.apoio_ciclos < CICLOS_MIN_PAPEL
        if travado or _adiantado(apoio) - _adiantado(ant) < VANTAGEM_TROCA:
            apoio = ant
    if ant is not None and ant != apoio:
        diagnostico.log("[TR] troca=apoio sai=r%s d_sai=%.0f entra=r%s d_entra=%.0f"
                        % (ant, dist(aliados[ant], ball) if ant in aliados else -1,
                           apoio, dist(aliados[apoio], ball)))
    memoria.apoio_ciclos = memoria.apoio_ciclos + 1 if apoio == ant else 0
    memoria.apoio = apoio

    papeis = {portador: PAPEL_PORTADOR, apoio: PAPEL_APOIO}
    for rid in linha:
        if rid not in papeis:
            papeis[rid] = PAPEL_COBERTURA
    return papeis


def eleger_buscador(mundo: Mundo, papeis: Dict[int, str]) -> Optional[int]:
    """With no carrier, the defender that goes for the ball: the closest one.

    Distance in 500 mm bands, ties by id, so two robots at similar distances do not
    swap every tick (raw closest took the crowding from 36% to 63% of the time).
    """
    if PAPEL_PORTADOR in papeis.values():
        return None
    melhor, melhor_chave = None, None
    for rid in mundo.linha:
        chave = (round(dist(mundo.aliados[rid], mundo.bola) / 500.0), rid)
        if melhor_chave is None or chave < melhor_chave:
            melhor, melhor_chave = rid, chave
    return melhor
