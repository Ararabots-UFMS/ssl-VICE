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


# ===========================================================================
#  QUEM CONTORNA E O PLANEJADOR, NAO A TATICA
# ===========================================================================
#
# ISTO SUBSTITUI O ARCO FEITO A MAO (ver RAIO_ORBITA abaixo), e a razao e a que
# o Felipe apontou em 09/10/2026: "o contorno e inerente a movimentacao, ela ja
# faz isso". E verdade, e o caminho estava DESLIGADO por nos:
#
#   movement_interfaces/msg/PlanningOptions.msg tem 'avoid_ball';
#   local_planner/obstacle_factory.py:64 transforma a bola em obstaculo
#       GenericCircleObstacle(bola, 60), e o padding default do
#       GenericCircleObstacle e 90 (o raio do robo) - ou seja, o planejador
#       mantem o CENTRO do robo a 150 mm do centro da bola e desvia com um
#       caminho continuo, replanejado de forma coerente;
#   tatics/running.py:1230 fazia 'cmd.ball = (papel != PAPEL_PORTADOR)' -
#       justamente para o PORTADOR a bola NAO era obstaculo.
#
# Ou seja: desligavamos o desvio do planejador e reimplementavamos "dar a volta"
# na tatica, com um alvo que pulava 0,9 rad por ciclo. Os sintomas vistos no
# lote do portador (09/10) sao os dessa escolha, nao de ajuste de constante:
# tremor, giro incompleto, bater na bola com a LATERAL em vez da placa, e errar
# a bola de raspao.
#
# O DESENHO AGORA TEM DUAS FASES, e o que decide e geometria pura:
#
#   CONTORNAR  o robo nao esta atras da bola em relacao ao alvo. O destino e um
#              ponto FIXO atras da bola, na linha de tiro, fora do obstaculo
#              (ver APROXIMACAO_ATRAS), e a bola entra como obstaculo. O
#              planejador faz a volta inteira - curta ou longa, nao e assunto
#              nosso.
#   EMPURRAR   o robo ja esta no corredor atras da bola. A bola deixa de ser
#              obstaculo e o alvo passa a ser ALEM dela (EMPURRAO), que e o que
#              atravessa a janela de disparo.
#
# POR QUE O ALVO DA FASE DE CONTORNO NAO DEPENDE DE ONDE O ROBO ESTA: era essa
# dependencia que tremia. O arco era calculado a partir do angulo ATUAL do robo,
# entao cada ciclo pedia um ponto diferente e o rastreador replanejava para um
# lugar novo 10 vezes por segundo. O ponto de espera depende so da bola e da
# mira - ele fica parado enquanto o robo se mexe.

# O planejador mantem o centro do robo a 60 + 90 = 150 mm do centro da bola, e
# 'adaptDestination' empurra para 180 mm qualquer destino que caia dentro disso.
# O ponto de espera tem de ficar FORA, senao o proprio planejador o desloca e o
# robo para num lugar que a tatica nao escolheu.
RAIO_BOLA_PLANEJADOR = 180.0
# Ponto de espera atras da bola: fora do obstaculo do planejador (180), com
# folga para ele chegar vindo de qualquer lado.
#
# JA FOI 450, E VOLTOU PARA 300 POR MEDICAO. Com 450 e a regra de "nao contornar
# dentro de 400 mm", a aproximacao final ficava impossivel: o robo so podia se
# aproximar se JA estivesse alinhado dentro de 20 graus, e no caminho de volta
# ele chegava a 95 mm quase alinhado, a regra o jogava para fora, e ele
# orbitava. Medido em 'portador_bola_lado_esq': pmin 95 mm (a placa e 94), bola
# andou 27 mm em 24,5 s, 1689 graus de giro, nenhum chute - contra +4248 mm e
# chute na configuracao de 300.
APROXIMACAO_ATRAS = 300.0
# O CORREDOR E EM MILIMETROS, NAO EM GRAUS.
#
# DEFEITO QUE ISTO CORRIGE, e ele explica "ele erra para onde vai e nao vai
# atras da bola". O teste era angular - 0,35 rad (20 graus) entre a direcao
# bola->robo e "exatamente atras da bola". Angulo nao e erro: o MESMO angulo
# vale desvios laterais completamente diferentes conforme a distancia.
#
#     distancia a bola     desvio lateral que 20 graus aceita
#               150 mm                     51 mm
#               300 mm                    103 mm
#               400 mm                    137 mm
#               700 mm                    240 mm
#              1200 mm                    411 mm
#              4000 mm                   1372 mm
#              6675 mm                   2289 mm
#
# O contato robo-bola e a 111 mm (casco 90 + bola 21). Ou seja: a partir de uns
# 700 mm o teste ja declarava "estou atras da bola" com o robo FORA do alcance
# de contato, e a 4 m aceitava 1,4 m de erro. Entrando na fase de empurrar
# nessa condicao, o robo segue em frente - e passa LONGE da bola.
#
# MEDIDO no lote do destino longo (09/10/2026): 'portador_longe_atras' terminou
# a 811 mm da linha de tiro, pmin 160 mm, ZERO por cento de contato - rapido,
# liso, e sem nunca tocar na bola.
#
# Agora o criterio e o desvio lateral em mm, que e a grandeza que decide se
# seguir em frente acerta a bola: 90 mm (o raio do casco) para entrar, 160 para
# sair. Em graus isso se aperta sozinho com a distancia, que e o que se quer.
DESVIO_ENCAIXE = 90.0
DESVIO_SOLTA = 160.0


# Encostado na bola e do lado errado, a primeira coisa a fazer e SAIR DE CIMA
# DELA. 300 mm: fora do obstaculo do planejador (180) com folga para ele achar
# caminho a partir dali.
#
# A TENTATIVA DE SAIR MAIS LARGO (450 mm, com "nao contornar dentro de 400")
# FOI MEDIDA E DESCARTADA. A ideia era por a volta fora da faixa em que a berma
# do planejador (39 mm) e menor que o erro de rastreio (p90 70 mm). O efeito
# colateral matou o ganho: com a aproximacao final barrada por raio, o robo
# passava a precisar estar JA alinhado para encostar.
#
#     configuracao          atras_150   lado_esq        diagonal   longe_atras
#     espera 300 / sai 180      -2680      +4248           +4745        +6049
#     espera 300 / sai 450      -1886         -3               -             -
#     espera 450 / sai 450      -1777        +27               -             -
#
# O caso que sobra - 'portador_bola_atras_150', unico em que o robo NASCE
# dentro do obstaculo da bola - nao se resolve daqui: qualquer rota do lado
# errado para o lado certo passa rente a bola, e a berma de 39 mm e menor que o
# erro de execucao. A correcao e no pacote movement (raio do obstaculo da bola
# em 60; o do robo adversario e 200, com o comentario "90 + 90 + 20").
RAIO_DESENCOSTA = 300.0


def fase_de_aproximacao(rx, ry, bx, by, dir_alvo, fase_anterior=None):
    """'desencostar', 'contornar' ou 'empurrar', por geometria pura.

    'fase_anterior' aplica a histerese: quem ja esta empurrando segue
    empurrando ate sair do corredor largo.
    """
    a_alvo = atan2(dir_alvo[1] - by, dir_alvo[0] - bx)
    ux, uy = cos(a_alvo), sin(a_alvo)
    # Desvio LATERAL do robo em relacao a reta bola->alvo, e de que lado da bola
    # ele esta. Para empurrar, as duas coisas: perto da reta E atras da bola.
    lateral = abs((rx - bx) * uy - (ry - by) * ux)
    atras = ((rx - bx) * ux + (ry - by) * uy) < 0.0
    limite = DESVIO_SOLTA if fase_anterior == "empurrar" else DESVIO_ENCAIXE
    if atras and lateral <= limite:
        return "empurrar"

    # ENCOSTADO E DO LADO ERRADO: SAI DE CIMA DA BOLA PRIMEIRO.
    #
    # DEFEITO QUE ISTO CORRIGE, medido no replay de 'portador_bola_lado_esq'
    # (09/10/2026), e ele era pior que tudo o que existia antes:
    #
    #     t= 2 s  robo a 141 mm da bola, indo nela
    #     t= 4 s  bola empurrada 660 mm para o NOSSO campo, robo a 103 mm
    #     t=14 s em diante  d = 55 a 68 mm - a bola DENTRO do casco - e os dois
    #             viajam juntos ate (-4161, 2334), o nosso canto
    #     resultado: avanco -4156 mm, 41% do tempo em contato
    #
    # O laco se fecha assim: com a bola encostada, "va para um ponto 300 mm
    # ATRAS da bola" e um ponto atras do PROPRIO ROBO, porque a bola esta em
    # cima dele. Ele anda para tras, a bola vem grudada (o grSim nao resolve a
    # sobreposicao, so empurra), e a cada ciclo o alvo recua de novo. O robo
    # leva a bola para o nosso fundo sem nunca errar uma ordem.
    #
    # Enquanto o centro do robo estiver dentro do obstaculo da bola, a unica
    # ordem que nao a arrasta e RADIAL: afastar-se na direcao em que ele ja
    # esta. Fora dali o planejador assume e faz a volta.
    if hypot(rx - bx, ry - by) < RAIO_BOLA_PLANEJADOR:
        return "desencostar"
    return "contornar"


def ponto_de_desencoste(rx, ry, bx, by, raio=RAIO_DESENCOSTA):
    """Afasta-se da bola pela linha que ja liga os dois - nunca a atravessa."""
    ang = atan2(ry - by, rx - bx)
    return no_campo(bx + cos(ang) * raio, by + sin(ang) * raio)


def ponto_de_espera(bx, by, dir_alvo, recuo=APROXIMACAO_ATRAS):
    """O ponto atras da bola na linha de tiro - destino da fase de contorno.

    Nao depende da posicao do robo, de proposito (ver o cabecalho da secao).
    """
    a_alvo = atan2(dir_alvo[1] - by, dir_alvo[0] - bx)
    d = max(recuo, RAIO_BOLA_PLANEJADOR + 60.0)
    return no_campo(bx - cos(a_alvo) * d, by - sin(a_alvo) * d)


def destino_de_empurrao(bx, by, dir_alvo):
    """Na fase de empurrar, o destino e o PROPRIO ALVO - atraves da bola.

    POR QUE ISTO SUBSTITUI O ALVO CONTINUO (ponto_de_aproximacao)
    ------------------------------------------------------------
    O alvo continuo depende da POSICAO DO ROBO em todos os seus termos: 'c' vem
    da distancia dele a bola, 't' do angulo de chegada dele, o desvio lateral do
    lado em que ele esta. Consequencia: o destino FOGE enquanto ele persegue.
    
    MEDIDO (09/10/2026), recomputando a decisao sobre os quadros gravados, a
    10 Hz como a estrategia:

        cenario       alvo que depende da posicao   alvo que nao depende
        lado_esq      17 mm/ciclo  (p90 59)          6 mm/ciclo (p90 26)
        longe_atras   17 mm/ciclo  (p90 86)          6 mm/ciclo (p90 55)
        diagonal      18 mm/ciclo  (p90 48)          7 mm/ciclo (p90 38)

    Os 6 mm sao o tremor da propria visao: e o piso. Ou seja, dois tercos do
    movimento do destino eram NOSSOS, e o planejador recebia um problema novo a
    cada 100 ms.

    E HA O SEGUNDO EFEITO, que e o maior: alvo curto limita a velocidade por
    construcao. Perfil do solver (a = 1500 mm/s^2, teto 2000 mm/s), medido:

        alvo a  300 mm   pico  671 mm/s   media  335
        alvo a  450 mm   pico  822        media  411
        alvo a 1600 mm   pico 1549        media  775
        alvo a 4000 mm   pico 2000        media 1200
        alvo a 6675 mm   pico 2000        media 1429

    Pedir "va 300 mm e pare" dez vezes por segundo e o que produz os 251 mm/s
    medidos em campo - e e a diferenca com a GUI, onde um clique a 4 m da 1200
    mm/s de media com UM plano, executado inteiro.

    O destino aqui depende so da bola e da mira, e fica longe: a linha reta
    bola->alvo, que e exatamente por onde a bola tem de sair. Atravessar a bola
    deixa de ser um truque de alvo e passa a ser o caminho.
    """
    return no_campo(dir_alvo[0], dir_alvo[1])


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
