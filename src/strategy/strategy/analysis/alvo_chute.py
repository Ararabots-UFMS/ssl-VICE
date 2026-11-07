"""Where the ball should go this tick: the goal, a team-mate, a clearance, or nowhere.

Every direction the roles use (body orientation, approach side, push point) comes
from this one decision, so it is made once per tick and locked against flicker.
"""

import time as _time
from dataclasses import dataclass
from math import atan2, hypot
from typing import Dict, Optional, Tuple

from strategy import diagnostico
from strategy.analysis.mundo import Mundo
from strategy.roles.atribuicao import PAPEL_APOIO, PAPEL_COBERTURA, PAPEL_PORTADOR
from strategy.skills import avaliacao_chute, chute, experimento, geometria, posicionamento

# The chosen aim holds for 2 s, like the roles. Switching goal<->pass every 10.9
# ticks spun our robots at twice the opponent's rate near the ball.
CICLOS_MIN_MIRA = 20
# Inside our defensive third (field 9000, goal at -4500) clearing beats building.
TERCO_DEFENSIVO = 3000.0
LIMITE_Y = 2600.0
# An opponent this close to the ball or to the carrier is pressing it.
PRESSAO_RAIO = 400.0
PRESSAO_MIN = 1
# A shot at the goal fires only with the body inside the gap it aims at. 20 deg let
# kicks fire 4-10 deg off a 2-5 deg gap, into the keeper (4 of 6 saves, 21/10/2026).
# Never below 2 deg, the body's p90 error at the kick, or every kick would wait.
TOL_MIN_VAO = 0.035
# Half a degree inside each edge: about the ball's own width seen from 3 m.
MARGEM_VAO = 0.009
# TROCA FAVORAVEL: de quanto a nota tem de melhorar para quebrar a trava.
#
# A trava (CICLOS_MIN_MIRA) existe por medicao: a mira alternando passe/
# bloqueado 132/148 ciclos girava a linha de tiro e o portador orbitava a bola
# sem encostar nela (07/10/2026). Entao a troca NAO pode ser por qualquer
# melhora - seria desligar a trava e trazer o giro de volta.
#
# Duas saidas, as duas conservadoras:
#   1. a mira travada saiu de qualquer vao aberto - ela virou impossivel, e
#      insistir nela e chutar no goleiro;
#   2. a nota de agora supera a nota DE QUANDO a travada foi adotada por esta
#      margem - melhora material, nao ruido.
#
# 0,15 e meia faixa entre o corte de baixa chance (0,1) e o de chute direto
# (0,3): menor que isso e flutuacao do proprio amostrador de 81 raios.
GANHO_MIN_TROCA = 0.15
# Tolerancia em y para dizer que a mira travada "ainda cabe" num vao: metade da
# largura da bola. Sem folga, ruido de 1 mm no amostrador derrubaria a trava.
MARGEM_FORA = 25.0
# PISO DE NOTA PARA CHUTAR AO GOL (chave NOTA_MIN_DISPARO).
#
# POR QUE: no lote de 20 contra goleiro varrendo (06/11/2026) a nota no
# disparo tinha mediana 0,424, e ela NAO separava resultado - gol 0,424,
# defendido 0,398. Ou seja ele disparava em vao mediocre e o goleiro chegava.
# O corte de fabrica e NOTA_CHUTE_DIRETO = 0,3, herdado de quando nao havia
# goleiro adversario no teste.
#
# 0,45 fica logo acima da mediana observada, entao cerca de metade dos
# disparos de hoje passariam a esperar. O risco e oposto e real: esperar
# demais virou 'sem_chute' 10 em 200 no lote de bola rolando. Por isso tem
# chave propria e lote medido - nao e numero de fe.
NOTA_MIN_DISPARO = 0.45


def _piso_nota():
    """O corte de nota para chutar ao gol neste ciclo."""
    if experimento.desligado("NOTA_MIN_DISPARO"):
        return avaliacao_chute.NOTA_CHUTE_DIRETO
    return NOTA_MIN_DISPARO

Ponto = Tuple[float, float]


@dataclass
class MemoriaMira:
    tipo: Optional[str] = None
    ponto: Optional[Ponto] = None
    tolerancia: Optional[float] = None  # kick tolerance of the held aim, rad
    ciclos: int = 0
    crua: Optional[str] = None          # this tick's choice before the lock
    segurada: bool = False              # the lock kept last tick's aim
    nota: Optional[float] = None        # nota do vao quando ESTA mira foi adotada
    # Pass receiver frozen at decision time as (x, y, rid): a moving receiver made
    # the push point jump every tick and the carrier never lined up (0 of 1272 frames).
    alvo_passe: Optional[Tuple[float, float, int]] = None


@dataclass(frozen=True)
class AlvoChute:
    ponto: Optional[Ponto]
    tipo: Optional[str]          # gol | passe | saida | alivio | bloqueado
    # Aim borrowed from last tick over a 'bloqueado': keep pointing, never arm.
    emprestada: bool
    receptor: Optional[Tuple[float, float, int]]
    # How far off the aim the body may be when the kick fires; None = the default.
    tolerancia: Optional[float] = None


def _tolerancia(ball, ax, ay, vao):
    """Angular room between the aim and the nearer edge of its gap, kept in bounds."""
    if vao is None or experimento.desligado("DISPARO_PELO_VAO"):
        return None
    a = atan2(ay - ball.position_y, ax - ball.position_x)
    # Normalised: attacking -x the goal sits near +-pi, where raw differences wrap.
    folga = min(geometria.norm_ang(a - vao[0]), geometria.norm_ang(vao[1] - a)) - MARGEM_VAO
    return max(TOL_MIN_VAO, min(chute.TOL_CORPO_DISPARO, folga))


# A ultima mira ja relatada, para o probe nao repetir a mesma linha 60x/s.
_ultimo_relato = [None]


def _diag_mira(nota_gol) -> None:
    """Mostra a nota trabalhando: TODOS os vaos, o escolhido e os descartados.

    POR QUE ISTO EXISTE: a nota e o vao escolhido nao apareciam em lugar
    nenhum. Com goleiro adversario em campo, "ele chutou no canto errado" e
    "ele achou o unico vao que havia" produzem o mesmo CSV, e sem ver os
    candidatos descartados nao da para separar os dois.
    """
    if not diagnostico.ativo():
        return
    u = avaliacao_chute.ULTIMO_VAO
    if not u:
        return
    # So quando a decisao MUDA: a 60 Hz a mesma linha sairia 60 vezes por
    # segundo e o arquivo deixaria de ser legivel.
    marca = (u.get("mira"), round(float(u.get("nota") or 0.0), 2), len(u.get("vaos") or ()))
    if marca == _ultimo_relato[0]:
        return
    _ultimo_relato[0] = marca
    decisao = ("direto" if nota_gol > avaliacao_chute.NOTA_CHUTE_DIRETO else
               "baixa_chance" if nota_gol > avaliacao_chute.NOTA_CHUTE_BAIXA_CHANCE else
               "bloqueado")
    forcada = experimento.mira_y(None)
    # RELOGIO DE PAREDE NA LINHA, e nao so a ordem.
    #
    # Sem ele quem le o arquivo nao sabe QUAL decisao foi a do disparo, e pegar
    # a ultima linha e errado: ela costuma ser depois do chute, com a bola ja
    # na linha do gol. Medido em 06/11/2026 (campanha_nota3 t2): a ultima linha
    # dizia nota=0.184 mira=489 com a bola em (4272,693), enquanto a decisao do
    # chute era nota=0.498 mira=188. time.time() porque e o mesmo relogio do
    # processo da campanha, que roda no mesmo container.
    diagnostico.log_e_arquivo(
        "[MR] t=%.3f bola=%.0f,%.0f abertas=%s/%s nota=%.3f mira=%s decisao=%s%s vaos=%s"
        % (_time.time(), u["bola"][0], u["bola"][1], u.get("abertas"), u.get("amostras"),
           float(u.get("nota") or 0.0), u.get("mira"), decisao,
           "" if forcada is None else " FORCADA=%.0f" % forcada,
           " | ".join("y=%+.0f nota=%.3f %s"
                      % (v["y"], v["nota"], "ESCOLHIDO" if v["escolhido"] else "descartado")
                      for v in (u.get("vaos") or ())) or "nenhum"))


def _cru(mundo: Mundo, papeis: Dict[int, str], memoria: MemoriaMira):
    """Goal if the line is clear, else the APOIO (or COBERTURA), else nothing.

    74% of our kicks had an opponent on the line within 2 m: kicking into them
    gives the ball back.
    """
    ball, aliados, inimigos = mundo.bola, mundo.aliados, mundo.inimigos
    bx, by = ball.position_x, ball.position_y
    gol_x, gol_y, nota_gol, vao = avaliacao_chute.melhor_vao(ball, mundo.gol_ataque, inimigos)
    _diag_mira(nota_gol)
    if nota_gol > _piso_nota():
        memoria.alvo_passe = None
        y = experimento.mira_y(gol_y)
        return gol_x, y, "gol", _tolerancia(ball, gol_x, y, vao)

    def _recebe(robo):
        return (posicionamento.pode_receber(robo, bx, by, mundo.gol_ataque)
                and geometria.linha_livre(bx, by, robo.position_x, robo.position_y,
                                          inimigos))

    congelado = memoria.alvo_passe
    if congelado is not None:
        atual = aliados.get(congelado[2])
        if (atual is not None and papeis.get(congelado[2]) in (PAPEL_APOIO, PAPEL_COBERTURA)
                and _recebe(atual)):
            return congelado[0], congelado[1], "passe", None
        memoria.alvo_passe = None

    # The receiver must be ahead of the ball: a retreating APOIO made passes go back.
    for papel_receptor in (PAPEL_APOIO, PAPEL_COBERTURA):
        for rid, papel in papeis.items():
            if papel != papel_receptor or rid not in aliados:
                continue
            a = aliados[rid]
            if _recebe(a):
                memoria.alvo_passe = (a.position_x, a.position_y, rid)
                return a.position_x, a.position_y, "passe", None

    # O RAMO DE BAIXA CHANCE TAMBEM RESPEITA O PISO.
    #
    # Sem isto o piso nao faria nada na campanha: sem companheiro em campo a
    # tentativa de passe falha, cai aqui, e 0,1 libera o mesmo chute que o
    # piso acabou de recusar.
    if nota_gol > max(avaliacao_chute.NOTA_CHUTE_BAIXA_CHANCE, _piso_nota()
                      if not experimento.desligado("NOTA_MIN_DISPARO") else 0.0):
        memoria.alvo_passe = None
        return gol_x, gol_y, "gol", _tolerancia(ball, gol_x, gol_y, vao)
    return None, None, "bloqueado", None


def _mira_existe(tipo, papeis, memoria: MemoriaMira) -> bool:
    """The goal never moves; a pass needs its receiver still in a receiving role."""
    if tipo != "passe":
        return True
    receptor = memoria.alvo_passe
    if receptor is not None and papeis.get(receptor[2]) == PAPEL_COBERTURA:
        return True
    return any(p == PAPEL_APOIO for p in papeis.values())


def _troca_favoravel(memoria: MemoriaMira, ant_p):
    """A mira travada deve ser abandonada agora? Devolve o motivo, ou None.

    Le o vao recem-calculado em avaliacao_chute.ULTIMO_VAO - este mesmo ciclo,
    por '_cru'. Nao recalcula nada: so compara.
    """
    u = avaliacao_chute.ULTIMO_VAO
    if not u or ant_p is None:
        return None
    vaos = u.get("vaos") or ()
    y_ant = ant_p[1]
    # 1. a mira travada saiu de todos os vaos abertos
    dentro = any(v["y_lo"] - MARGEM_FORA <= y_ant <= v["y_hi"] + MARGEM_FORA
                 for v in vaos)
    if not dentro:
        return "travada_bloqueada"
    # 2. a nota de agora supera a de quando a travada foi adotada
    nota_ant = memoria.nota
    nota_hoje = float(u.get("nota") or 0.0)
    if nota_ant is not None and nota_hoje >= nota_ant + GANHO_MIN_TROCA:
        return "nota_subiu"
    return None


_ultimo_uso = [None]


def _diag_uso(y, tipo, segurada, ciclos) -> None:
    """A mira EM VIGOR, depois da trava - que nao e a avaliacao do ciclo.

    POR QUE SEPARADO DO [MR]: '_mira_firme' segura a mira anterior por ate
    CICLOS_MIN_MIRA ciclos, entao no instante do disparo a mira em uso pode
    ser mais VELHA que o melhor_vao daquele ciclo. Medido em 06/11/2026
    (campanha_nota3 t1): o [MR] do chute dizia mira=-252 e a bola cruzou em
    +465. Sem esta linha nao da para saber se o atacante errou a mira dele ou
    se mirou numa mira travada de 20 ciclos atras.
    """
    if not diagnostico.ativo():
        return
    marca = (None if y is None else round(y), tipo, bool(segurada))
    if marca == _ultimo_uso[0]:
        return
    _ultimo_uso[0] = marca
    diagnostico.log_e_arquivo(
        "[MU] t=%.3f uso_y=%s tipo=%s segurada=%s ciclos=%s"
        % (_time.time(), None if y is None else round(y, 1), tipo, int(bool(segurada)), ciclos))


def _mira_firme(memoria: MemoriaMira, ax, ay, tipo, tol, papeis):
    """Hold the previous valid aim for CICLOS_MIN_MIRA ticks, also over a 'bloqueado'.

    'bloqueado' is the absence of information, not news: alternating passe/bloqueado
    (132/148 ticks measured) left the carrier orbiting the ball without touching it.
    """
    ant_t, ant_p, n = memoria.tipo, memoria.ponto, memoria.ciclos
    ant_vale = ant_t not in (None, "bloqueado") and ant_p is not None

    if experimento.desligado("MIRA_FIRME"):
        segura = ant_vale and n < CICLOS_MIN_MIRA and tipo not in (None, "bloqueado")
    else:
        segura = ant_vale and n < CICLOS_MIN_MIRA and _mira_existe(ant_t, papeis, memoria)

    # TROCA FAVORAVEL: a trava solta quando a mira nova e materialmente melhor.
    #
    # MEDIDO (campanha_guarda20, 06/11/2026): 18 dos 20 chutes sairam com a
    # mira SEGURADA, a mira em vigor trocava a cada 2,18 s - o periodo da
    # trava - e a nota no disparo tinha mediana 0,424 contra um maximo de 1,0.
    # Com goleiro varrendo a 354 mm/s, 2,18 s sao ~770 mm, mais que a
    # meia-largura do gol: a mira seguia onde o goleiro ESTAVA, nao onde ele
    # esta. E a nota nao separava gol (0,424) de defesa (0,398).
    if segura and ant_t == "gol" and not experimento.desligado("TROCA_FAVORAVEL"):
        motivo = _troca_favoravel(memoria, ant_p)
        if motivo:
            segura = False
            memoria.ciclos = CICLOS_MIN_MIRA   # nao segura de novo no ciclo seguinte

    memoria.crua = tipo
    memoria.segurada = bool(segura)
    if segura:
        memoria.ciclos = n + 1
        _diag_uso(ant_p[1], ant_t, True, n + 1)
        return ant_p[0], ant_p[1], ant_t, memoria.tolerancia

    memoria.tipo = tipo
    memoria.ponto = None if ax is None else (ax, ay)
    memoria.tolerancia = tol
    # A NOTA DE QUANDO ESTA MIRA FOI ADOTADA, para a troca favoravel comparar
    # com a de agora em vez de com a do ciclo anterior.
    memoria.nota = (float(avaliacao_chute.ULTIMO_VAO.get("nota") or 0.0)
                    if tipo == "gol" and avaliacao_chute.ULTIMO_VAO else None)
    _diag_uso(ay, tipo, False, 0)
    memoria.ciclos = 0
    return ax, ay, tipo, tol


def _saida_de_bola(mundo: Mundo):
    """In our third with nothing open: clear to the side, half-way up the field.

    The best available line, not a clear one (requiring a clear line fired 2 times
    in 170). Aimed at the touch line so it stays in play: Aimless Kick, Division B.
    """
    ball = mundo.bola
    sent = 1.0 if mundo.gol_ataque.x >= 0 else -1.0
    ly = LIMITE_Y if ball.position_y >= 0 else -LIMITE_Y
    candidatos = [(ball.position_x + 1800.0 * sent, ly * 0.7),
                  (ball.position_x + 2200.0 * sent, 0.0),
                  (ball.position_x + 1800.0 * sent, -ly * 0.7)]
    melhor, melhor_folga = None, -1.0
    for cx, cy in candidatos:
        dx, dy = cx - ball.position_x, cy - ball.position_y
        n = hypot(dx, dy) or 1.0
        folga = geometria.folga_lateral(ball.position_x, ball.position_y,
                                        dx / n, dy / n, n, mundo.inimigos)
        if folga > melhor_folga:
            melhor, melhor_folga = (cx, cy), folga
    return melhor


def _alivio(mundo: Mundo, portador):
    """Pressed with no line: protect the ball by kicking away from whoever presses.

    Pressure is measured on the ball as well as the robot: behind the ball, the
    carrier is too far from a frontal press for its own radius to see it.
    """
    ball, inimigos = mundo.bola, mundo.inimigos

    def _perto(e, alvo):
        return hypot(e.position_x - alvo.position_x, e.position_y - alvo.position_y) < PRESSAO_RAIO

    if experimento.desligado("PRESSAO_BOLA"):
        perto = sum(1 for e in inimigos.values() if _perto(e, portador))
    else:
        perto = sum(1 for e in inimigos.values() if _perto(e, ball) or _perto(e, portador))
    if perto < PRESSAO_MIN:
        return None

    ameaca = min(inimigos.values(),
                 key=lambda e: hypot(e.position_x - portador.position_x,
                                     e.position_y - portador.position_y))
    if experimento.desligado("PROTECAO"):
        lat_y = LIMITE_Y if ball.position_y >= 0 else -LIMITE_Y
        avanco = 800.0 if mundo.gol_ataque.x >= 0 else -800.0
        alvo = (ball.position_x + avanco, lat_y)
    else:
        alvo = posicionamento.saida_sob_pressao(
            ball.position_x, ball.position_y, inimigos, mundo.gol_ataque,
            mundo.nosso_gol, ameaca=ameaca)
    if alvo is not None:
        diagnostico.log("[JG] ALIVIO perto=%d -> %.0f,%.0f" % (perto, alvo[0], alvo[1]))
    return alvo


def alvo_do_chute(mundo: Mundo, papeis: Dict[int, str], memoria: MemoriaMira) -> AlvoChute:
    ax, ay, tipo, tol = _cru(mundo, papeis, memoria)
    ax, ay, tipo, tol = _mira_firme(memoria, ax, ay, tipo, tol, papeis)
    ponto = None if ax is None else (ax, ay)
    emprestada = bool(memoria.segurada and memoria.crua in (None, "bloqueado"))

    ball = mundo.bola
    d_nosso_gol = hypot(ball.position_x - mundo.nosso_gol.x,
                        ball.position_y - mundo.nosso_gol.y)
    if d_nosso_gol < TERCO_DEFENSIVO and tipo in (None, "bloqueado"):
        saida = _saida_de_bola(mundo)
        if saida is not None:
            ponto, tipo, tol = saida, "saida", None
        diagnostico.log("[JG] SAIDA d_gol=%.0f -> %s" % (d_nosso_gol, tipo))

    if ponto is None and mundo.inimigos:
        rid_p = next((k for k, v in papeis.items() if v == PAPEL_PORTADOR), None)
        if rid_p is not None and rid_p in mundo.aliados:
            alivio = _alivio(mundo, mundo.aliados[rid_p])
            if alivio is not None:
                ponto, tipo, tol = alivio, "alivio", None

    diagnostico.log("[JG] alvo_chute=%s inimigos=%d situacao=%s"
                    % (tipo, len(mundo.inimigos or {}), mundo.situacao))
    return AlvoChute(ponto=ponto, tipo=tipo, emprestada=emprestada,
                     receptor=memoria.alvo_passe, tolerancia=tol)
