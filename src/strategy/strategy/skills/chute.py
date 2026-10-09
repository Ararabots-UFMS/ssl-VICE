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

from strategy.skills.geometria import norm_ang

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


# Dentro disto o corpo mira a direcao do chute; fora, olha para a bola.
RAIO_ORIENTA_CHUTE = 700.0
# Quanto a direcao do chute pode divergir de "onde a bola esta" antes de a ordem
# virar as COSTAS do robo para ela.
#
# EXATAMENTE 90 GRAUS, de proposito: assim o invariante e demonstravel - nenhuma
# ordem de orientacao deixa a bola atras do robo. Com 100 graus sobravam 25 de
# 576 posicoes na sonda, todas na faixa de 91 a 99 graus. Nao se perde nada:
# a janela do grSim exige |yy| < 40 mm (bola praticamente no eixo do corpo),
# entao a partir de ~90 graus nao existe chute possivel dali de qualquer forma.
TOL_LADO_CERTO = pi / 2


def orientacao_do_corpo(robo, ball, alvo_chute, gol_ataque_xy,
                        so_lado_certo=True):
    """Para onde o corpo deve apontar. Devolve (angulo, mira_o_chute).

    DEFEITO QUE ISTO CORRIGE - "o robo vira a bunda para a bola"
    ------------------------------------------------------------
    A regra era: a menos de 700 mm da bola, aponte na direcao
    atan2(alvo - bola). Essa direcao e calculada no referencial da BOLA, igual
    para qualquer posicao do robo - entao METADE do circulo em volta dela fica
    do lado errado dela, e nessas posicoes a ordem manda o robo ficar de costas
    para a bola.

    MEDIDO (sonda de decisao offline, varrendo o robo em volta da bola em 36
    posicoes, quatro cenarios, raios de 120 a 900 mm):

        raio 120, 260 e 500 mm   costas em 18 de 36 posicoes  (exatamente metade)
        raio 900 mm              costas em  0 de 36
        erro maximo              180 graus (costas completas)

    A 900 mm nao acontece porque ali a regra ja era "olhe para a bola". Ou seja:
    nao e um caso de borda sob pressao, e metade das posicoes, sempre, por
    construcao.

    A CORRECAO mantem o que foi medido como certo - quem chega pelo lado certo
    mira a direcao do chute, porque o tiro sai no eixo do corpo - e muda so o
    lado errado: ali o robo olha para a bola, o que o deixa contornar sem perder
    o contato de vista. Ver tambem aproximacao.ponto_de_aproximacao, que leva o
    robo a dar a volta em vez de empurrar do lado errado.
    """
    d = hypot(robo.position_x - ball.position_x, robo.position_y - ball.position_y)
    mira = alvo_chute if alvo_chute is not None else gol_ataque_xy
    ang_saida = atan2(mira[1] - ball.position_y, mira[0] - ball.position_x)
    ang_para_bola = atan2(ball.position_y - robo.position_y,
                          ball.position_x - robo.position_x)
    if d >= RAIO_ORIENTA_CHUTE:
        return ang_para_bola, False
    if not so_lado_certo:
        # COMPORTAMENTO ANTIGO (chave ARARABOTS_SEM_ORIENTACAO_LADO): a direcao
        # do chute sempre, sem olhar de que lado do robo a bola esta. Metade das
        # posicoes em volta dela viram as costas.
        return ang_saida, True
    # do lado certo? entao o corpo assume a linha de tiro
    if abs(norm_ang(ang_para_bola - ang_saida)) < TOL_LADO_CERTO:
        return ang_saida, True
    # lado errado: a bola ficaria nas costas. Olha para ela e contorna.
    return ang_para_bola, False


def chutar_em(robo, ball, ponto, tipo_alvo, sentido_ataque, travas, rid,
              gol_ataque_xy=None, so_lado_certo=True, pode_armar=True):
    """"Chute a bola NAQUELE ponto": os tres canais, numa chamada.

    Devolve (angulo, armado, forca) - o que a estrategia precisa pedir para que
    a bola saia na direcao de 'ponto'. O ALVO DE MOVIMENTO nao sai daqui: ele
    depende do papel e da situacao, e vem de
    aproximacao.ponto_de_aproximacao(..., dir_alvo=ponto).

    POR QUE ISTO NAO ERA "EXPRESSAVEL" ANTES, e o que mudou
    ------------------------------------------------------
    Chutar num ponto exige TRES canais com destinos diferentes:
        posicao     -> topico movement_manager/commands (planejador)
        orientacao  -> servico set_orientation (pacote control)
        chute       -> servico update_kick     (pacote control)
    Nenhum deles, isolado, chuta. E ha uma quarta condicao, que nao esta em
    nenhuma API: control.py:120,139 so publica o TeamCommand - que leva o kick e
    a orientacao - enquanto o rastreador emite referencia, isto e ENQUANTO O
    ROBO ESTA INDO A ALGUM LUGAR. Robo que chega ao alvo cala o canal e o chute
    fica preso no cache, armado, sem nunca ser enviado (medido: 'arma=True' 34 a
    110 vezes por lote, ZERO disparos, com o robo a 92-96 mm da bola).
    
    O INVARIANTE, portanto, e: o alvo de movimento nunca pode ser alcancavel.
    Quem garante isso e ponto_de_aproximacao, que mira ALEM da bola
    (ATRAVESSA_CHUTE) ou no arco de contorno - nunca no ponto de chute exato.
    Este docstring existe para que a proxima pessoa nao "melhore" isso mirando o
    ponto certo e perca o chute de novo.
    
    Enquanto o laco do control nao for corrigido, esta funcao e o lugar unico
    onde essa dependencia esta escrita. Quando for corrigido, e aqui que o
    comentario sai - e nao em tres taticas diferentes.
    """
    gol = gol_ataque_xy if gol_ataque_xy is not None else (
        (ball.position_x + 1000.0 * sentido_ataque, ball.position_y))
    angulo, _mira = orientacao_do_corpo(robo, ball, ponto, gol, so_lado_certo)
    # 'pode_armar' SEPARA A MIRA DO GATILHO.
    #
    # A mira e segurada por 2 s para o corpo e a aproximacao nao girarem (ver
    # running.mira_firme). O GATILHO nao pode ser segurado junto: 'armar_chute'
    # nao confere linha - a unica protecao contra chutar no adversario e
    # 'alvo_do_chute' devolver None - e o defeito medido era justamente chutar
    # em quem estava no caminho (14 de 19 instantes de chute, 74%). Entao
    # quando a mira deste ciclo e a EMPRESTADA do ciclo anterior, o robo
    # continua chegando e apontando para o alvo, mas nao arma.
    armado = pode_armar and armar_chute(
        robo, ball, ponto, sentido_ataque, tipo_alvo, travas, rid)
    return angulo, armado, (forca_por_alvo(tipo_alvo) if armado else 0.0)


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
