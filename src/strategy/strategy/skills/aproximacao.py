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

from strategy.skills.bola import onde_a_bola_vai, velocidade
from strategy.skills.geometria import no_campo, norm_ang, tempo_de_chegada

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
# Uma corda de 45 graus neste raio ainda deixa o casco longe da bola.
RAIO_CONTORNO_SEGURO = 220.0
TOL_APROXIMACAO_SEGURA = 0.35

# Ate quanto tempo no futuro vale procurar um ponto de bloqueio. Acima disso a
# bola ja teria atravessado o campo inteiro (9000 mm a 900 mm/s = 10s, mas um
# chute de verdade e muito mais rapido - ver skills/bola.py).
HORIZONTE_BLOQUEIO = 2.0  # s
PASSO_BLOQUEIO = 0.05     # s

# Teto de velocidade PLAUSIVEL para a bola. Acima disto e ruido do filtro, nao
# chute real.
#
# MEDIDO (e e por isso que isto existe): num teleporte de cenario de teste, a
# bola salta de posicao e o Kalman que alimenta /game_state interpreta o salto
# como velocidade - vimos leitura de ate 28000 mm/s por uma fracao de segundo,
# contra um chute real de ~5900 mm/s no maximo documentado (skills/chute.py).
# Sem o teto, essa leitura extrapola um alvo a centenas de mm da trajetoria
# real - foi o que mandou a cobertura para y>1000 enquanto a bola ia por
# y<450. So acontece em teleporte: num jogo de verdade a bola nunca salta de
# posicao, mas o teto fica porque QUALQUER filtro pode ter um pico de ruido, e
# o custo de errar para o lado conservador (cair para 'cobertura_na_linha') e
# bem menor que o de confiar cegamente num numero absurdo.
VELOCIDADE_BOLA_MAX_PLAUSIVEL = 6500.0  # mm/s


def ponto_de_bloqueio_a_tempo(rx, ry, ball, nosso_gol,
                              horizonte=HORIZONTE_BLOQUEIO,
                              passo=PASSO_BLOQUEIO):
    """Ponto na trajetoria da bola que o robo alcanca ANTES (ou junto) dela.

    POR QUE ISTO PRECISOU EXISTIR
    ------------------------------
    'posicionamento.cobertura_na_linha' planta o robo numa FRACAO FIXA do
    caminho bola->nosso-gol, pela posicao ATUAL da bola - nunca pela
    velocidade dela. O ponto pode estar geometricamente certo e ainda assim
    ser inalcancavel a tempo, porque o robo tambem gasta tempo para chegar
    la.

    MEDIDO no cenario 'cobertura_chute_longe' (bola a 1900-6000 mm/s vinda do
    meio-campo, ninguem nosso sobre a linha de tiro): com a fracao fixa,
    4 a 5 de 6 execucoes terminam em GOL CONTRA, sem relacao clara entre
    velocidade da bola e resultado - o padrao esperado de uma defesa que
    acerta por coincidencia geometrica, nao por calculo.

    O QUE FAZ: varre a trajetoria prevista da bola (posicao + velocidade
    constante - ela nao tem motor, so desacelera, e desprezar isso so torna a
    estimativa mais conservadora) em passos de tempo, e devolve o PRIMEIRO
    ponto em que o 'tempo_de_chegada' do robo e <= o tempo da bola chegar la.
    E o ponto mais proximo da bola - logo, o mais cedo - que o robo ainda
    alcanca a tempo.

    DUAS GUARDAS, as duas para nao desviar a cobertura atras de bola que nao
    ameaca:
      - bola abaixo do limiar de 'chutada' (ver skills/bola.py): devolve None,
        nao ha o que interceptar por tempo numa bola rolando devagar.
      - velocidade nao aponta para o NOSSO gol: devolve None. Sem isso, uma
        bola saindo de perto (ex: o proprio reposicionamento dela num cenario
        de teste) faz a cobertura correr atras de um alvo que nao e ameaca.

    Devolve None se nenhuma das duas guardas passar, OU se nenhum ponto do
    horizonte e alcancavel - quem chama entao cai para 'cobertura_na_linha'.
    """
    vx, vy = velocidade(ball)
    speed = hypot(vx, vy)
    if speed < 1.0:
        return None
    if speed > VELOCIDADE_BOLA_MAX_PLAUSIVEL:
        escala = VELOCIDADE_BOLA_MAX_PLAUSIVEL / speed
        vx, vy, speed = vx * escala, vy * escala, VELOCIDADE_BOLA_MAX_PLAUSIVEL

    bx, by = ball.position_x, ball.position_y
    dist_agora = hypot(bx - nosso_gol.x, by - nosso_gol.y)
    # a bola precisa estar se aproximando do NOSSO gol, nao so se movendo
    aproximando = (vx * (nosso_gol.x - bx) + vy * (nosso_gol.y - by)) > 0
    if not aproximando:
        return None

    t = passo
    while t <= horizonte:
        bx_t, by_t = bx + vx * t, by + vy * t
        if hypot(bx_t - nosso_gol.x, by_t - nosso_gol.y) >= dist_agora:
            break  # a bola ja passou do ponto mais proximo do gol - sem sentido seguir
        if tempo_de_chegada(rx, ry, bx_t, by_t) <= t:
            return no_campo(bx_t, by_t)
        t += passo
    return None


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


def ponto_de_aproximacao_segura(robo, bx, by, dir_alvo):
    """Contorna sem atravessar a bola; gira antes de avancar para o contato.

    A cobertura pode receber a bola pelo lado oposto ao passe. O ajuste de
    chegada da aproximacao continua elimina o contorno perto da bola e, nesse
    caso, empurra a bola com o corpo ainda virado para a propria meta.
    """
    rx, ry = robo.position_x, robo.position_y
    mira = atan2(dir_alvo[1] - by, dir_alvo[0] - bx)
    radial = atan2(ry - by, rx - bx)
    erro_posicao = norm_ang(mira + pi - radial)
    erro_corpo = abs(norm_ang(robo.orientation - mira))
    distancia = hypot(rx - bx, ry - by)
    if abs(erro_posicao) > TOL_APROXIMACAO_SEGURA:
        # Primeiro sai do contato; depois contorna em cordas curtas que nao
        # cruzam a bola, mesmo quando o planejador nao a trata como obstaculo.
        passo = max(-pi / 4, min(pi / 4, erro_posicao)) if distancia >= 180.0 else 0.0
        angulo = radial + passo
        return no_campo(bx + RAIO_CONTORNO_SEGURO * cos(angulo),
                        by + RAIO_CONTORNO_SEGURO * sin(angulo))
    if erro_corpo > TOL_APROXIMACAO_SEGURA:
        if distancia < 180.0:
            return no_campo(bx + RAIO_CONTORNO_SEGURO * cos(radial),
                            by + RAIO_CONTORNO_SEGURO * sin(radial))
        return rx, ry
    return ponto_de_aproximacao(rx, ry, bx, by, dir_alvo, AVANCO_SOLTA)
