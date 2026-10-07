"""Chegar na bola: contorno continuo, ponto de chute e interceptacao.

CAMADA DE SKILLS - nivel 2.

POR QUE ESTE MODULO EXISTE
--------------------------
Chegar na bola estava escrito tres vezes, com tres desenhos diferentes:
fases discretas em tatics/freekick.py (contornar / encaixar / empurrar), alvo
continuo interpolado em tatics/running.py, e um ponto 70 mm atras da bola em
tatics/goalkeeper.py. O desenho continuo e o mais medido dos tres e e o que
mora aqui.

AS DUAS LICOES QUE ESTE MODULO CARREGA, ambas pagas em lote de teste:

1. O LADO POR ONDE SE CHEGA E O QUE DECIDE O CHUTE. Sem dribbler, a bola sai na
   direcao robo->bola. Medido no portao, 50 ciclos de posse nossa: a bola estava
   na face do chutador em 94 de 96 ciclos, mas o corpo apontava para o NOSSO gol
   em 82 deles - chegamos pelo lado errado e a trava de seguranca barrou o
   disparo, sobrando trombada.

2. NUNCA "CHEGAR". O kick viaja dentro do TeamCommand, e control.py:120 so
   publica esse TeamCommand enquanto o rastreador emite referencia - isto e,
   enquanto o robo esta indo a algum lugar. Mirar exatamente o ponto de chute
   fazia o robo CHEGAR, o rastreador se calar e o chute ficar preso no cache,
   armado, sem nunca ser enviado: medido, 'arma=True' 34 a 110 vezes por lote e
   ZERO disparos, com o robo a 92-96 mm da bola (dentro da janela).
   Por isso o alvo proximo e ALEM da bola, nao o ponto de chute.
"""

from math import atan2, cos, hypot, pi, sin

from strategy.skills.bola import onde_a_bola_vai
from strategy.skills.geometria import no_campo, norm_ang

# Onde o CENTRO do robo precisa estar para a bola cair na placa: 73 (placa) + 21
# (raio da bola). Medido nos replays: chegamos a 95 e 99 mm, dentro da janela,
# mas em UM unico quadro de cada partida - passamos de raspao em vez de
# estacionar ali.
PONTO_CHUTE = 94.0
# Distancia em que a aproximacao deixa de valer velocidade e passa a valer
# precisao. Dentro disto o alvo vira a travessia da janela de disparo.
RAIO_ENCAIXE = 600.0
# Quanto o alvo recua para TRAS da bola quando a chegada esta desalinhada.
RECUO_CONTORNO = 700.0
# Desvio lateral do contorno em arco, longe da bola. Zera na chegada.
LATERAL_CONTORNO = 550.0
# Vies lateral calibrado da cadeia de movimento (o robo chega deslocado deste
# tanto; o alvo compensa).
VIES_LATERAL = -27.0
# Onde o alvo fica na chegada, medido A PARTIR DA BOLA na direcao de saida.
# Negativo = AQUEM dela (do nosso lado); positivo = ALEM dela.
#
# -53 e o valor de "ainda nao da para empurrar": a chegada esta torta, o robo
# encosta de raspao e o alvo nao o manda atravessar a bola.
ATRAVESSA_CHUTE = -53.0

# ALEM DA BOLA QUANDO A CHEGADA ESTA ALINHADA - o "empurrao".
#
# DEFEITO QUE ISTO CORRIGE, medido em 07/10/2026 com sonda guiada por replay
# (as funcoes desta camada sao puras; os quadros vem dos replays do lote de
# 03/10). Em TODOS os quadros de contato medidos o alvo estava 53 mm AQUEM da
# bola:
#
#     orbita_frontal  antes  off mediana -58,3 mm   alem da bola em  0% dos quadros
#     orbita_frontal  depois off mediana -56,1 mm   alem da bola em  3%
#     orbita_colado   depois off mediana -53,0 mm   alem da bola em  9%
#     pressao_na_bola        off mediana -54,9 mm   alem da bola em  0%
#
# O casco tem 90 mm e a bola 21: o CENTRO do robo nao chega a menos de 111 mm
# do centro da bola. Um alvo a -53 mm e inalcancavel por construcao, e o que
# sobra e um erro residual de ~58 mm. Com kp = 2,3 (control/pid_controller.py:8)
# isso da 2,3 x 0,058 = 0,13 m/s - e o feedforward esta desligado pelo ajuste do
# PID, que e a condicao de medida desta fase. O robo encosta na bola e a bola
# nao sai do lugar: medido no replay, deslocamento LIQUIDO da bola de 0 a 8 mm
# em 24,5 s nos tres cenarios de protecao.
#
# E o proprio cabecalho deste modulo ja dizia o certo ("por isso o alvo proximo
# e ALEM da bola, nao o ponto de chute") - a constante contradizia o texto.
#
# 180 mm: com o robo em contato, o alvo fica a 111 + 180 = 291 mm dele, o que da
# 2,3 x 0,291 = 0,67 m/s de comando. Empurrao de verdade, e trajeto que sobra de
# folga para o rastreador nao se calar (ver o cabecalho).
EMPURRAO = 180.0
# So empurra quando a chegada esta BOA. t e o alinhamento (1 = perfeito), entao
# 0,80 equivale a ~36 graus de erro entre a nossa chegada e a direcao de saida.
# Abaixo disso o alvo continua aquem da bola e a geometria tem tempo de melhorar
# - empurrar torto e mandar a bola para o lado errado, que e o defeito que a
# orbita existe para evitar.
TOL_EMPURRAO = 0.80
# Avanco alem da bola/ponto de interceptacao, por situacao.
AVANCO_PORTADOR = 500.0
AVANCO_SOLTA = 1600.0

# --- contorno em orbita: pegar a bola POR TRAS quando ela esta atras de nos ---
#
# 260 mm de raio: fora do contato (casco 90 + bola 21 = 111 mm) e dentro do
# RAIO_ENCAIXE, para nao disputar com o regime de longe. Com o passo de ~52
# graus, a corda do arco passa a 260*cos(26) = 234 mm da bola - o robo da a
# volta sem encostar nela no caminho.
RAIO_ORBITA = 260.0
# O alvo fica sempre ~52 graus a frente do robo no arco. E tambem o que mantem o
# robo EM MOVIMENTO: alvo alcancavel cala o rastreador e, com ele, o canal de
# chute (ver o cabecalho deste modulo).
PASSO_ORBITA = 0.9             # rad


def ponto_de_interceptacao(rx, ry, ball, avanco, segundos=0.5):
    """Onde ir quando a bola JA esta viajando: o ponto previsto, e alem dele.

    Com a bola em movimento nao ha o que contornar - o lado por onde se chega
    deixa de ser escolha nossa e passa a ser a trajetoria dela. Mirar alem do
    ponto previsto evita chegar freando (ver skills/bola.py).
    """
    fx, fy = onde_a_bola_vai(ball, segundos)
    dx, dy = fx - rx, fy - ry
    n = hypot(dx, dy) or 1.0
    return no_campo(fx + (dx / n) * avanco, fy + (dy / n) * avanco)


def ponto_de_aproximacao(rx, ry, bx, by, dir_alvo, avanco_base, contornar=True,
                         empurrar=True):
    """Alvo continuo para chegar na bola pelo lado certo e atravessa-la.

    'dir_alvo' e o ponto para onde a bola deve sair (gol, companheiro, lateral).
    O alvo desliza sem degrau:
        desalinhado -> um ponto ATRAS da bola (contorna)
        alinhado    -> um ponto ALEM dela (atravessa e chuta)
    Sem degrau de proposito: a versao com fronteira fixa fazia o robo orbitar o
    limite - ele cruzava, o alvo pulava 500 mm, ele recuava, e ficava oscilando
    sem se comprometer (medido: 'arma' nao fechou nenhuma vez em 166 ciclos, com
    a distancia ao alvo nunca descendo de 202 mm).
    """
    a_alvo = atan2(dir_alvo[1] - by, dir_alvo[0] - bx)
    a_cheg = atan2(by - ry, bx - rx)
    t = 1.0 - min(abs(norm_ang(a_cheg - a_alvo)) / pi, 1.0)   # 1 = perfeito
    ux_a, uy_a = cos(a_alvo), sin(a_alvo)
    d_ate_bola = hypot(rx - bx, ry - by)

    # A BOLA ESTA ATRAS DE NOS? ENTAO CONTORNA E PEGA POR TRAS.
    #
    # DEFEITO QUE ISTO CORRIGE. Perto da bola o alvo era o ponto logo antes
    # dela NA LINHA DE TIRO, e o desvio lateral do contorno zerava (de
    # proposito: na chegada o robo tem de estar EM CIMA da linha). Com o robo do
    # lado errado, esse alvo so e alcancavel atravessando a bola - e sem
    # dribbler a bola sai na direcao robo->bola, isto e, PARA TRAS.
    #
    # MEDIDO (sonda de decisao offline, 12 largadas em volta da bola x 3 raios x
    # 2 cenarios, deixando a propria decisao guiar o robo por 40 passos):
    #     termina do lado errado em 22 de 72 largadas (31%)
    #     pior caso: largada em cima da linha de tiro, 175 graus de erro -
    #     ele empurrava a bola de volta para o nosso campo
    #
    # A correcao e geometrica: o robo precisa estar ATRAS da bola em relacao ao
    # alvo (a_alvo + pi). Se ele nao esta e ja esta perto, o alvo passa a ser um
    # ponto do ARCO em volta da bola, um passo adiante na direcao mais curta -
    # ele orbita ate chegar do lado certo, e so entao o regime normal assume.
    ang_robo = atan2(ry - by, rx - bx)
    delta = norm_ang(norm_ang(a_alvo + pi) - ang_robo)
    if contornar and d_ate_bola < RAIO_ENCAIXE and abs(delta) > pi / 2:
        passo = delta if abs(delta) < PASSO_ORBITA else (
            PASSO_ORBITA if delta > 0 else -PASSO_ORBITA)
        a_novo = ang_robo + passo
        return no_campo(bx + cos(a_novo) * RAIO_ORBITA,
                        by + sin(a_novo) * RAIO_ORBITA)

    # PERTO, ATRAVESSA A JANELA; LONGE, MIRA ALEM DA BOLA.
    #
    # Mirar 500-1600 mm alem da bola faz o robo chegar rapido e bater com o
    # ombro: a bola escapa antes do disparo. Os dois regimes num alvo continuo,
    # interpolados pela distancia.
    c = min(max((d_ate_bola - PONTO_CHUTE) / (RAIO_ENCAIXE - PONTO_CHUTE),
                0.0), 1.0)
    # NA CHEGADA, ALEM DA BOLA SE ESTIVER ALINHADO (ver EMPURRAO).
    #
    # Sem degrau, como todo o resto deste modulo: o alvo desliza de -53 mm
    # (chegada torta) a +180 mm (chegada perfeita) conforme o alinhamento.
    off_perto = ATRAVESSA_CHUTE
    if empurrar:
        k = max(0.0, (t - TOL_EMPURRAO) / (1.0 - TOL_EMPURRAO))
        off_perto = ATRAVESSA_CHUTE + (EMPURRAO - ATRAVESSA_CHUTE) * k
    off_alinhado = off_perto + (avanco_base - off_perto) * c
    # PERTO DA BOLA, COMPROMETE-SE COM A JANELA.
    #
    # O termo de contorno dependia so do alinhamento. Com o robo a 90 mm da bola
    # e alinhamento imperfeito (t = 0,6), o alvo virava 333 mm ATRAS da bola
    # estando colado nela - e ele recuava. Media disso: 5 ciclos armados por
    # replay. O lado por onde contornar ja foi escolhido la longe; na chegada o
    # alvo e o meio da janela de disparo, e ponto.
    off_longe = -RECUO_CONTORNO * (1.0 - t) + off_alinhado * t
    off = c * off_longe + (1.0 - c) * off_perto

    # O DESVIO LATERAL TEM DE SUMIR AO CHEGAR.
    #
    # Ele dependia so do alinhamento: perto da bola, com alinhamento imperfeito,
    # seguia empurrando o alvo 550 mm para o lado - e o robo passava AO LADO da
    # bola. Medido no referencial do robo: em 15 ciclos armados 'dispara' foi
    # False nos 15, com yy de -73 a -260 mm contra um limite de 40, sempre do
    # mesmo lado e crescendo. A bola ficava no ombro, nunca na placa.
    cruz = ux_a * (ry - by) - uy_a * (rx - bx)
    lado = 1.0 if cruz >= 0 else -1.0
    lat = LATERAL_CONTORNO * (1.0 - t) * lado * c + VIES_LATERAL
    return no_campo(bx + ux_a * off - uy_a * lat,
                    by + uy_a * off + ux_a * lat)
