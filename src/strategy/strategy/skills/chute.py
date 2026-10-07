"""Chutar: a janela de disparo do grSim, a trava de armamento e a forca.

CAMADA DE SKILLS - nivel 2.

POR QUE ESTE MODULO EXISTE
--------------------------
Este e o caso mais caro de conhecimento duplicado do pacote. A bola parada
descobriu, pagando lotes de teste, tres coisas que o jogo corrido teve de
redescobrir meses depois:

  1. a janela de disparo do grSim dura UMA amostra, entao recalcular a condicao
     a cada ciclo faz o chute PISCAR e acertar o quadro do contato vira sorte -
     medimos 73 pedidos de chute e ZERO disparos, com o comando comprovadamente
     chegando ao grSim;
  2. o teste do grSim e SIMETRICO (usa fabs no eixo do corpo), entao uma bola
     encostada nas COSTAS do robo satisfaz o mesmo criterio - e um companheiro
     viu em campo o robo chutando de traseira;
  3. nao se exige alinhamento fino no instante do contato: medido, 'mirado'
     falso em 96 de 96 ciclos com a bola, e o time nunca chutou.

Referencia obrigatoria antes de mexer aqui: documentacao/estrategia/CHUTE.md.

A GEOMETRIA, no referencial do robo (grSim robot.cpp:120-128):

    xx = distancia da bola ao longo do eixo do corpo, menos a placa
    yy = desvio lateral da bola em relacao ao eixo do corpo
    dispara se  0 <= xx < 31,5 mm  E  |yy| < 40 mm

O casco tem raio 90 mm e a bola 21: tocando o casco REDONDO a distancia trava em
111 mm e o chutador nunca alcanca a bola. Quem bate com o ombro nunca chuta.
"""

from math import atan2, cos, hypot, pi, sin

from strategy.skills.geometria import (
    FOLGA_LINHA, folga_lateral, livre_do_lado, no_campo, norm_ang,
)

# --- a geometria do chutador, medida no grSim -----------------------------
CENTRO_ATE_PLACA = 73.0
ESPESSURA_PLACA = 5.0
RAIO_BOLA_MM = 21.5
LIM_XX_GRSIM = ESPESSURA_PLACA * 2.0 + RAIO_BOLA_MM     # 31,5 mm
LIM_YY_GRSIM = 40.0

# --- o portao do jogo corrido --------------------------------------------
# Arma com a bola a menos disto. 260 mm e folgado de proposito: o simulador
# ignora o comando ate haver contato, entao armar cedo nao custa nada e recusar
# armar custa a jogada inteira.
ALCANCE_CONTATO = 260.0
# Uma vez armado, segue armado ate a bola passar disto. E a trava que a bola
# parada ja tinha e o jogo corrido nao: sem ela o chute pisca na aproximacao.
ALCANCE_SOLTA_TRAVA = 420.0
# Distancia maxima para sequer considerar armar.
FORCA_CHUTE_ALCANCE = 300.0
# Tolerancia de "a bola nao esta nas minhas costas": a projecao no eixo do corpo
# pode ficar 20 mm atras da placa (ruido de visao), nao mais.
MARGEM_FRENTE_PLACA = -20.0
# Quanto a direcao de saida pode divergir do ataque sem virar chute para a
# propria meta. 1,75 rad ~ 100 graus para cada lado.
TOL_PARA_TRAS = 1.75

# --- forca por tipo de alvo, em m/s --------------------------------------
# 6,0 no gol: a cobranca de falta mediu o atrito do grSim - 3,0 m/s percorre
# 1633 mm, 5,0 percorre 1803 e 6,0 percorre 4220. A curva e abrupta, e abaixo de
# 6 a bola morre antes de atravessar o campo. O teto da regra 8.4.2 e 6,5.
FORCA_CHUTE = 6.0
# 2,5 no passe: forca de gol atravessa o receptor. 2,5 cobre ~1,5 m, que e a
# distancia tipica do apoio.
FORCA_PASSE = 2.5
# 4,5 na saida de bola: media, para a bola PARAR no campo deles e nao sair pela
# linha de fundo - Aimless Kick, Division B.
FORCA_SAIDA = 4.5


def alvo_de_afastamento(ball, nosso_gol, inimigos):
    """Afasta pelo corredor de maior folga, mesmo quando todos estao ocupados."""
    bx, by = ball.position_x, ball.position_y
    sentido = -1.0 if nosso_gol.x > 0 else 1.0
    candidatos = []
    for graus in range(-75, 76, 15):
        ang = graus * pi / 180.0
        x, y = no_campo(bx + sentido * 2200.0 * cos(ang),
                       by + 2200.0 * sin(ang))
        dx, dy = x - bx, y - by
        alcance = hypot(dx, dy)
        # Toda a trajetoria deve aumentar a distancia da nossa meta.
        if (alcance < 300.0 or dx * sentido <= 0
                or (bx - nosso_gol.x) * dx + (by - nosso_gol.y) * dy < 0):
            continue
        folga = folga_lateral(bx, by, dx / alcance, dy / alcance,
                             alcance, inimigos)
        livre = folga >= FOLGA_LINHA and livre_do_lado(x, y, inimigos)
        candidatos.append((livre, folga,
                           hypot(x - nosso_gol.x, y - nosso_gol.y), x, y))
    if not candidatos:
        return None
    _, _, _, x, y = max(candidatos)
    return x, y


def geometria_do_chutador(robo, ball):
    """(xx, yy, frente) da bola no referencial do corpo do robo.

    xx e yy reproduzem robot.cpp:120-128, medidos a partir da PLACA.
    'frente' e a projecao COM SINAL de robo->bola no eixo do corpo, e NAO existe
    no grSim: e ela que distingue a bola na placa da bola nas costas.
    """
    dx, dy = cos(float(robo.orientation)), sin(float(robo.orientation))
    bx = ball.position_x - robo.position_x
    by = ball.position_y - robo.position_y
    frente = bx * dx + by * dy
    return frente - CENTRO_ATE_PLACA, -bx * dy + by * dx, frente


def na_janela_de_disparo(xx, yy):
    """O grSim dispararia NESTE instante?"""
    return 0.0 <= xx < LIM_XX_GRSIM and abs(yy) < LIM_YY_GRSIM


def forca_por_alvo(tipo_alvo):
    """Forca do chute conforme o que se quer fazer com a bola."""
    if tipo_alvo == "passe":
        return FORCA_PASSE
    if tipo_alvo == "saida":
        return FORCA_SAIDA
    return FORCA_CHUTE


def direcao_para_frente(ball, alvo_chute, sentido_ataque):
    """A bola sairia na direcao do ataque (e nao da nossa meta)?"""
    ang_saida = atan2(alvo_chute[1] - ball.position_y,
                      alvo_chute[0] - ball.position_x)
    referencia = 0.0 if sentido_ataque > 0 else pi
    return abs(norm_ang(ang_saida - referencia)) < TOL_PARA_TRAS


def corpo_pode_afastar(robo, ball, alvo_chute, nosso_gol, inimigos):
    """O eixo real do chutador afasta da meta com folga suficiente?

    O tiro sai pelo corpo, nao pela mira. Aceita um corredor livre diferente
    do alvo; sob pressao exige pelo menos a folga do melhor corredor escolhido.
    """
    if alvo_chute is None:
        return False
    ux, uy = cos(robo.orientation), sin(robo.orientation)
    bx, by = ball.position_x, ball.position_y
    sentido = -1.0 if nosso_gol.x > 0 else 1.0
    if ux * sentido <= 0 or (bx - nosso_gol.x) * ux + (by - nosso_gol.y) * uy < 0:
        return False
    dx, dy = alvo_chute[0] - bx, alvo_chute[1] - by
    alcance = hypot(dx, dy)
    if alcance < 1.0:
        return False
    folga_alvo = folga_lateral(bx, by, dx / alcance, dy / alcance,
                               alcance, inimigos)
    folga_corpo = folga_lateral(bx, by, ux, uy, alcance, inimigos)
    # Versores equivalentes podem diferir alguns ulps ao passar por atan2/cos.
    return folga_corpo + 1e-6 >= min(FOLGA_LINHA, folga_alvo)


def armar_chute(robo, ball, alvo_chute, sentido_ataque, tipo_alvo, travas, rid):
    """Decide se o chute deste robo fica ARMADO neste ciclo.

    'travas' e o dicionario de estado da jogada (estado["chute_armado"]): a
    decisao e stateful de proposito, porque a janela de disparo dura uma amostra.
    Quem arbitra o disparo continua sendo o grSim, que so dispara com a bola na
    placa - manter armado nao cria chute torto, so deixa de perder o chute certo.

    Regras, na ordem:
      sem alvo, ou bola longe        -> nao arma
      ja armado e ainda perto        -> SEGUE armado (a trava)
      bola na placa e direcao certa  -> arma
      bola nas costas                -> nunca arma
      direcao da nossa meta          -> so em passe
    """
    d_bola = hypot(robo.position_x - ball.position_x,
                   robo.position_y - ball.position_y)
    alcance = ALCANCE_SOLTA_TRAVA if travas.get(rid) else FORCA_CHUTE_ALCANCE
    if alvo_chute is None or d_bola >= alcance:
        travas[rid] = False
        return False

    # xx JA e a distancia a placa com sinal: negativo = bola atras da placa.
    frente_placa, _, _ = geometria_do_chutador(robo, ball)
    para_frente = direcao_para_frente(ball, alvo_chute, sentido_ataque)
    ok_dir = para_frente or tipo_alvo == "passe"
    perto = d_bola < ALCANCE_CONTATO and frente_placa > MARGEM_FRENTE_PLACA

    if (travas.get(rid) and d_bola < ALCANCE_SOLTA_TRAVA
            and ok_dir and frente_placa > MARGEM_FRENTE_PLACA):
        arma = True
    else:
        arma = perto and ok_dir
    travas[rid] = arma
    return arma
