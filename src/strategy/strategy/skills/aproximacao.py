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
# Quanto ALEM da bola o alvo fica na chegada. Negativo porque e medido a partir
# da bola na direcao de saida: ~53 mm antes dela, de modo que o trajeto restante
# (~175 mm) mantenha o rastreador vivo durante a travessia.
ATRAVESSA_CHUTE = -53.0
# Avanco alem da bola/ponto de interceptacao, por situacao.
AVANCO_PORTADOR = 500.0
AVANCO_SOLTA = 1600.0


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


def ponto_de_aproximacao(rx, ry, bx, by, dir_alvo, avanco_base):
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

    # PERTO, ATRAVESSA A JANELA; LONGE, MIRA ALEM DA BOLA.
    #
    # Mirar 500-1600 mm alem da bola faz o robo chegar rapido e bater com o
    # ombro: a bola escapa antes do disparo. Os dois regimes num alvo continuo,
    # interpolados pela distancia.
    d_ate_bola = hypot(rx - bx, ry - by)
    c = min(max((d_ate_bola - PONTO_CHUTE) / (RAIO_ENCAIXE - PONTO_CHUTE),
                0.0), 1.0)
    off_alinhado = ATRAVESSA_CHUTE + (avanco_base - ATRAVESSA_CHUTE) * c
    # PERTO DA BOLA, COMPROMETE-SE COM A JANELA.
    #
    # O termo de contorno dependia so do alinhamento. Com o robo a 90 mm da bola
    # e alinhamento imperfeito (t = 0,6), o alvo virava 333 mm ATRAS da bola
    # estando colado nela - e ele recuava. Media disso: 5 ciclos armados por
    # replay. O lado por onde contornar ja foi escolhido la longe; na chegada o
    # alvo e o meio da janela de disparo, e ponto.
    off_longe = -RECUO_CONTORNO * (1.0 - t) + off_alinhado * t
    off = c * off_longe + (1.0 - c) * ATRAVESSA_CHUTE

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
