from utils.math_util import Vector2D
import os

from math import atan2, hypot, pi, cos, sin

from strategy.skills.skills import Skills
from strategy.tatics.goalkeeper import Goalkeeper


# Forca do chute em jogo aberto, em m/s, e o alcance para arma-lo.
#
# 6,0 m/s: a cobranca de falta mediu o atrito do grSim - 3,0 m/s percorre
# 1633 mm, 5,0 percorre 1803, e 6,0 percorre 4220. A curva e abrupta, e abaixo
# de 6 a bola morre antes de atravessar o campo. O teto da regra 8.4.2 e 6,5.
FORCA_CHUTE = 6.0
# 300 mm: ARMA CEDO e mantem armado durante toda a aproximacao final.
#
# Era 130 mm (o contato acontece a 111) e o chutador do grSim NAO disparou em
# nenhuma execucao - 2 eventos registrados, zero disparos. O motivo esta no
# §18 da cobranca de falta: o grSim so dispara no instante em que a bola encosta
# na placa, e essa janela dura UMA amostra. Se o chute nao estiver armado
# exatamente nela, o que sobra e o empurrao do corpo.
#
# Armar cedo nao custa nada - o simulador ignora o comando ate haver contato.
# Recusar armar custa a jogada inteira.
FORCA_CHUTE_ALCANCE = 300.0

# Acima desta velocidade a bola JA FOI CHUTADA - ninguem persegue.
#
# POR QUE ISTO PRECISA EXISTIR - o defeito que segurava o jogo inteiro
# ---------------------------------------------------------------------
# O chute saia certo: MEDIDO no replay, com o robo atras da bola e o disparo
# confirmado pelo grSim (flat_kick em t=2,31), a bola partiu para a frente a
# 5929 mm/s. E percorreu 200 mm.
#
# O motivo esta no robot.cpp:168 do grSim: o contato SUBTRAI velocidade da bola
# (KickerDampFactor). Como o alvo do empurrao fica alem dela, o robo continua
# avancando depois do disparo, alcanca a bola e a FREIA. Medimos ele grudado a
# 158 mm dela pelos 22 s seguintes, com a bola imovel em x=-668.
#
# A cobranca de falta ja tinha passado por isto (§6.7 item 4: "chute de 6 m/s
# virando 350 mm") e resolveu com o recuo pos-chute. O jogo corrido nao tinha
# equivalente.
#
# O criterio e a VELOCIDADE DA BOLA, nao um estado guardado: a tatica e
# reconstruida a cada ciclo, entao qualquer memoria aqui seria perdida.
#
# 250 mm/s, e nao 800. O valor TEM de vir do que a estrategia enxerga, nao da
# fisica: a velocidade chega pelo filtro de Kalman, que subnotifica e atrasa (o
# HANDOVER da bola parada registra 6196 mm/s reais publicados como 1607).
#
# MEDIDO no proprio /game_state durante um jogo, 526 amostras:
#     p50 = 15    p90 = 36    max = 1321 mm/s
# Com o limiar em 800 a condicao so fechava no pico, por um ou dois quadros -
# quando o robo ja tinha alcancado a bola e freado. 250 fica muito acima do
# ruido (p90 = 36) e pega a bola ainda saindo.
VEL_BOLA_CHUTADA = 250.0


def bola_ja_saiu(ball):
    """A bola esta viajando? Entao nao ha o que empurrar - ver VEL_BOLA_CHUTADA."""
    vx = getattr(ball, "velocity_x", 0.0) or 0.0
    vy = getattr(ball, "velocity_y", 0.0) or 0.0
    return hypot(vx, vy) > VEL_BOLA_CHUTADA


def eleger_atacante(ally_robots, ball):
    """Quem vai buscar a bola: o mais proximo dela, excluindo o goleiro.

    POR QUE ISTO PRECISOU EXISTIR - o impasse fechado do jogo corrido
    -----------------------------------------------------------------
    Nem Atack nem Defense escolhiam alguem para ir a bola. Em Defense TODOS os
    robos de linha recebiam as MESMAS coordenadas (o ponto medio entre a bola e
    o nosso gol) e ainda com robot_command.ball = True, ou seja, com a bola
    marcada como OBSTACULO - o planejador desviava dela.

    Medido em 8 execucoes de 25 s do cenario 'jogo':
      - a bola nunca passou de x = -1194 (o gol deles fica em +4500);
      - ZERO quadros com um robo nosso alem de x = 3500;
      - pico da bola 1668 mm/s, ou seja, so esbarrao (chute real e 5000+);
      - 36% do tempo com DOIS robos nossos a menos de 600 mm da bola.

    O ciclo se fechava: bola no nosso campo -> a arvore escolhe Defense ->
    Defense nao vai a bola -> a bola nao sai do nosso campo -> Atack nunca liga.

    Eleger pelo mais proximo e o mesmo criterio do _eleger_cobrador da cobranca
    de falta, que ja se mostrou estavel: nao depende de estado, so da geometria
    do instante, entao nao ha o que travar.
    """
    # DISTANCIA QUANTIZADA, e nao a distancia crua.
    #
    # Eleger pelo "mais proximo" puro ALTERNA: com dois robos a distancias
    # parecidas, o vencedor muda a cada ciclo, os dois recebem ora "va a bola"
    # ora "cubra", e ambos terminam grudados nela.
    #
    # MEDIDO: com a eleicao crua o amontoado (dois nossos a menos de 600 mm da
    # bola) foi de 36% para 63% do tempo - pior que antes de existir eleicao.
    #
    # Arredondando a distancia em faixas de 500 mm e desempatando pelo ID, dois
    # robos "igualmente perto" dao sempre o mesmo vencedor: enquanto a diferenca
    # entre eles nao passar de uma faixa, a escolha nao se mexe. E estavel sem
    # guardar estado - e guardar estado aqui e caro, porque a tatica e
    # reconstruida a cada ciclo (o mesmo defeito do 'parent' que a cobranca de
    # falta ja teve).
    #
    # Nao e trava: nenhuma condicao precisa fechar para a jogada andar, e a
    # escolha muda assim que alguem fica MEIO METRO mais perto - que e uma
    # diferenca real, nao ruido.
    melhor, melhor_ch = None, None
    for rid, r in ally_robots.items():
        if rid == 0:                      # o goleiro nunca sai para buscar
            continue
        d = hypot(r.position_x - ball.position_x, r.position_y - ball.position_y)
        chave = (round(d / 500.0), rid)
        if melhor_ch is None or chave < melhor_ch:
            melhor, melhor_ch = rid, chave
    return melhor




# ==========================================================================
#  SITUACOES DE JOGO E PAPEIS
# ==========================================================================
#
# POR QUE ISTO EXISTE
# -------------------
# A tatica so distinguia "o eleito e os outros", e o eleito era sempre o mais
# proximo da bola: o time inteiro era funcao da posicao dela. Nao havia "eles
# estao com a bola", "a bola esta solta", "estamos disputando".
#
# MEDIDO com './ararabots.sh posse 6' - 8802 quadros, 6 execucoes:
#     DISPUTA  51%      NOSSA  33%      DELES  8%      SOLTA  8%
# e a bola no NOSSO campo em ~100% do tempo, nas quatro situacoes.
#
# Ou seja: o caso dominante era justamente o que nao tinha tratamento. A disputa
# era tratada como posse nossa, com todos indo a bola - dai um robo segurando o
# outro em cima dela metade do jogo.
RAIO_POSSE = 250.0        # contato fisico e 111 mm; 250 cobre "esta com ela"

SITUACAO_NOSSA = "NOSSA"
SITUACAO_DELES = "DELES"
SITUACAO_DISPUTA = "DISPUTA"
SITUACAO_SOLTA = "SOLTA"

PAPEL_PORTADOR = "portador"
PAPEL_APOIO = "apoio"
PAPEL_COBERTURA = "cobertura"


def situacao_de_jogo(ally_robots, enemy_robots, ball):
    """Quem esta com a bola AGORA. So geometria, sem estado guardado."""
    bx, by = ball.position_x, ball.position_y
    dn = min((hypot(r.position_x - bx, r.position_y - by)
              for rid, r in ally_robots.items() if rid != 0), default=9e9)
    dd = min((hypot(r.position_x - bx, r.position_y - by)
              for r in enemy_robots.values()), default=9e9) if enemy_robots else 9e9
    if dn <= RAIO_POSSE and dd <= RAIO_POSSE:
        return SITUACAO_DISPUTA
    if dn <= RAIO_POSSE:
        return SITUACAO_NOSSA
    if dd <= RAIO_POSSE:
        return SITUACAO_DELES
    return SITUACAO_SOLTA


def onde_a_bola_vai(ball, segundos=0.5):
    """Posicao prevista da bola daqui a 'segundos'.

    ATENCAO a fonte: a velocidade vem do filtro de Kalman, que SUBNOTIFICA e
    atrasa. MEDIDO no /game_state durante um jogo, 526 amostras: p50 = 15,
    p90 = 36, maximo 1321 mm/s - enquanto a visao crua mostra picos de 6000.
    A previsao, portanto, e conservadora por construcao: erra para perto, nunca
    para longe. Serve para escolher ONDE ESPERAR, nao para cronometrar nada.
    """
    vx = getattr(ball, "velocity_x", 0.0) or 0.0
    vy = getattr(ball, "velocity_y", 0.0) or 0.0
    return ball.position_x + vx * segundos, ball.position_y + vy * segundos


def _dist_bola(robo, ball):
    return hypot(robo.position_x - ball.position_x,
                 robo.position_y - ball.position_y)


def distribuir_papeis(ally_robots, ball, situacao, estado=None, sentido=1.0):
    """Quem busca a bola, quem ocupa o espaco a frente, quem cobre.

    QUEM BUSCA E QUEM ATACA SAO PAPEIS DIFERENTES - e antes eram o mesmo.
    ---------------------------------------------------------------------
    A eleicao antiga escolhia o mais PROXIMO da bola e o chamava de portador.
    Consequencia: quem recuperava a bola era sempre o mais adiantado, que por
    isso perdia a posicao de ataque. Toda recuperacao custava o nosso espaco.

    Agora:
      - PORTADOR   e o mais ADIANTADO. Ele ocupa o espaco a frente e recebe.
                   So vai a bola quando ela e nossa e esta com ele.
      - BUSCADOR   e quem vai buscar, escolhido ENTRE OS DE TRAS. Recuperar
                   deixou de custar a posicao ofensiva.
      - COBERTURA  fica entre a bola e o nosso gol.

    QUANTOS VAO A BOLA, por situacao (pedido do Felipe):
      SOLTA          UM  - o mais proximo entre os de tras. Dois atras da mesma
                     bola solta e desperdicio: o outro fica livre para o espaco.
      DELES/DISPUTA  DOIS - buscador e cobertura. Medimos 'bloqueado' + 'alivio'
                     em 200 de 242 ciclos porque eles chegam com tres e nos com
                     um; um contra tres nao se resolve posicionando.
      NOSSA          o buscador vira segundo atacante e sobe.

    'sentido' e +1 quando atacamos +x e -1 quando atacamos -x: e o que define
    "adiantado".
    """
    linha = sorted(rid for rid in ally_robots if rid != 0)
    if not linha:
        return {}

    def _adiantado(rid):
        return ally_robots[rid].position_x * sentido

    # QUEM BUSCA SAI PRIMEIRO, E E O MAIS PROXIMO DA BOLA.
    #
    # Antes o PORTADOR era escolhido primeiro, como o mais adiantado, e o
    # buscador saia dos que sobravam. Na LARGADA isso e desastroso: a bola fica
    # no centro, o nosso robo 1 esta a 1200 mm dela e os robos 2 e 3 a ~2500 mm.
    # O robo 1 virava portador - o mais adiantado - e ficava parado ocupando
    # espaco, enquanto mandavamos disputar quem estava MAIS LONGE. Medido: em
    # 2 de 3 replays o amarelo tocava a bola aos 0,7 s e chutava 5400 mm direto
    # para o nosso fundo, com UM unico toque na partida inteira.
    #
    # Invertendo a ordem, a mesma regra produz os dois comportamentos que
    # queremos, pela geometria:
    #   - bola no centro na largada -> o mais proximo e o da frente, e ele vai;
    #   - bola sobrando no nosso campo com o atacante la em cima -> o mais
    #     proximo e um dos de tras, e o portador fica livre para o espaco,
    #     que foi exatamente o pedido original.
    buscador = min(linha, key=lambda rid: _dist_bola(ally_robots[rid], ball))
    if estado is not None:
        ant = estado.get("buscador")
        if ant is not None and ant in linha and ant != buscador:
            travado = estado.get("buscador_ciclos", 0) < CICLOS_MIN_PAPEL
            if travado or (_dist_bola(ally_robots[ant], ball)
                           - _dist_bola(ally_robots[buscador], ball)
                           < VANTAGEM_TROCA):
                buscador = ant
        estado["buscador_ciclos"] = (
            estado.get("buscador_ciclos", 0) + 1 if buscador == ant else 0)
        estado["buscador"] = buscador

    # PORTADOR: o mais adiantado ENTRE OS QUE SOBRAM.
    sobram = [rid for rid in linha if rid != buscador]
    if not sobram:
        return {buscador: PAPEL_APOIO}
    portador = max(sobram, key=_adiantado)
    if estado is not None:
        ant = estado.get("portador")
        if ant is not None and ant in sobram and ant != portador:
            travado = estado.get("portador_ciclos", 0) < CICLOS_MIN_PAPEL
            if travado or _adiantado(portador) - _adiantado(ant) < VANTAGEM_TROCA:
                portador = ant
        estado["portador_ciclos"] = (
            estado.get("portador_ciclos", 0) + 1 if portador == ant else 0)
        estado["portador"] = portador

    papeis = {portador: PAPEL_PORTADOR, buscador: PAPEL_APOIO}
    for rid in linha:
        if rid not in papeis:
            papeis[rid] = PAPEL_COBERTURA
    return papeis


def _livre_do_lado(x, y, inimigos, folga=320.0):
    """Nenhum adversario a menos de 'folga' deste ponto."""
    return not any(hypot(e.position_x - x, e.position_y - y) < folga
                   for e in (inimigos or {}).values())


def _norm_ang(a):
    """Angulo em (-pi, pi]."""
    while a > pi:
        a -= 2.0 * pi
    while a <= -pi:
        a += 2.0 * pi
    return a


FOLGA_LINHA = 180.0

# PRESSAO: adversarios a menos de PRESSAO_RAIO do portador. 400 mm e pouco mais
# que um diametro de robo (180) - quem esta ai disputa o corpo, nao marca.
# Perto o bastante para ja assumir a linha de tiro em vez de olhar para a bola.
# Com a bola: perto o bastante para o chutador alcanca-la.
# Um novo candidato so vira portador se estiver isto mais perto que o atual.
# TROCAR DE PAPEL E CARO: so por vantagem GRANDE e clara.
#
# 600 mm ja tinha zerado a permuta em campo limpo, mas com o adversario em campo
# as trocas voltaram (2 de portador e 2 de apoio em 250 ciclos, e antes disso
# blocos de 3 a 13 ciclos alternando). Cada troca joga fora a corrida inteira
# que o robo ja fez: quem estava chegando abandona e outro comeca de longe.
#
# 1200 mm e mais que meio campo de distancia util - a troca passa a acontecer
# so quando o papel esta claramente com o robo errado, nao por disputa de
# alguns centimetros.
VANTAGEM_TROCA = 1200.0
# HISTERESE POR TEMPO, alem da margem de distancia.
#
# A margem sozinha nao basta: quando dois robos se cruzam, a diferenca de
# distancia passa por zero e a troca acontece de qualquer jeito. Vail & Veloso
# (CMU, 2003) descrevem o mecanismo padrao da liga - uma vez que um robo assume
# um papel ele NAO o abandona por um curto periodo, "da ordem de segundos" - e
# observam que as funcoes de decisao sao auto-reforcantes: quem assume o papel
# tende a ficar mais apto a ele.
#
# A estrategia roda a 10 Hz, entao 20 ciclos = 2 s de posse do papel.
CICLOS_MIN_PAPEL = 20
# A mira segura o tipo escolhido por 2 s, como os papeis. Ver a nota em
# alvo_do_chute sobre o giro medido.
CICLOS_MIN_MIRA = 20
RAIO_POSSE_PORTADOR = 200.0
# Dentro disto o portador disputa a bola sempre, independente do rotulo da
# situacao - que oscila com o rastreio. 450 mm e o dobro do raio de posse.
ENGAJA_RAIO = 450.0
# Quanto ele avanca ALEM da bola ao empurrar/tocar - atravessa em vez de parar.
AVANCO_PORTADOR = 500.0
# ATAQUE A BOLA SOLTA, em forca total.
#
# Medido nos replays: com a bola solta os nossos andavam a p50 de 38 a 192 mm/s
# estando a 1200-2500 mm dela. O planejador desacelera ao se aproximar do alvo,
# entao mirar A BOLA significa chegar nela devagar - e quem chega devagar na
# bola solta perde a bola solta.
#
# Mirando MUITO alem dela, o alvo nunca esta perto e a desaceleracao nao entra:
# o robo cruza a bola em velocidade. E o "vai igual maluco" na pratica, dentro
# do unico canal que temos (o 'aggressiveness' da mensagem nao e lido pelo
# planejador - ver PLANO.md, item M2).
AVANCO_SOLTA = 1600.0
# Contorno: quanto o alvo fica ATRAS da bola quando a chegada esta no lado
# errado, e quanto ele se desloca para o lado para o robo fazer a curva.
RECUO_CONTORNO = 700.0
# Onde o CENTRO do robo precisa estar para a bola encostar na placa do chutador:
# 73 mm (placa) + 21 mm (raio da bola). Alem disso o casco redondo bate antes.
PONTO_CHUTE = 94.0
# COMPENSACAO DE VIES, medida e nao arbitrada.
#
# Com o alvo pedindo xx=15 e |yy| centrado, o que se mede nos ciclos armados e:
#     xx  mediana  50   -> o robo para 35 mm ALEM do pedido
#     yy  mediana -50   -> sempre para o MESMO lado
#
# O yy nao e dispersao, e VIES: metade dos ciclos ja cairia dentro da janela se
# a distribuicao estivesse centrada. Entao pedimos deslocado, de proposito, para
# o erro de rastreio cair DENTRO em vez de fora.
#
#   lateral: -50 mm, contrario ao vies medido
#   longitudinal: pedir 53 mm em vez de 88. O robo nao cabe a 53 (a placa fica
#   a 73), entao ele PRESSIONA para dentro e para onde a fisica deixa - na
#   placa, que e exatamente onde queremos.
# CALIBRADO, nao arbitrado. Com -50 a mediana do yy foi de -50 para +42, ou
# seja o deslocamento moveu a distribuicao em +92 mm e passou do centro. Para
# levar -50 ate 0, o valor proporcional e -27.
#
# O fato de o deslocamento MOVER a mediana de forma previsivel prova que o
# desvio lateral e VIES, e nao ruido de rastreio - vies se corrige mirando
# deslocado, ruido nao.
VIES_LATERAL = -27.0
# A partir daqui o alvo comeca a virar o ponto de chute em vez do avanco.
# OPCAO A: CHEGAR MAIS DEVAGAR.
#
# O xx medido tem espalhamento de 7 a 179 mm com mediana 92 - dispersao, nao
# vies (o yy, que era vies, cedeu na primeira calibracao: mediana -50 -> -12 e
# a taxa dentro da janela subiu de 18% para 67%). Deslocar o alvo nao corrige
# dispersao.
#
# Mas houve ciclos com xx=7 e xx=44: o robo CHEGA na janela as vezes, so nao
# fica. Comecando a mirar o ponto de chute de 1200 mm em vez de 600, ele passa
# mais tempo desacelerando e chega com menos velocidade - menos ultrapassagem,
# menos espalhamento.
# TESTADO E REVERTIDO: 1200 (chegar mais devagar) PIOROU.
#
#   com 600:   armados 6, xx mediana  92, xx na janela 1, yy_ok 4
#   com 1200:  armados 2, xx mediana 106, xx na janela 0, yy_ok 2
#
# Comecar a mirar o ponto de chute mais cedo nao reduziu a ultrapassagem: so
# reduziu o tempo em que o robo fica perto o bastante para armar. A dispersao do
# xx (7 a 179 mm) nao vem da velocidade de chegada.
RAIO_ENCAIXE = 600.0
# Alvo proximo: ALEM da bola, nao no ponto de chute. Ver a nota sobre o
# control.py:120 - robo parado significa comando de chute nao enviado.
# Alvo proximo: pouco ALEM do ponto de chute, nao muito alem da bola.
#
# HISTORICO, para nao repetir: mirar o PONTO DE CHUTE (-94, onde a face encosta)
# deu a melhor geometria ja medida - 25 a 52 quadros por partida dentro da
# janela, contra 1 antes. Eu troquei isso por +80 (alem da bola) acreditando que
# o robo parado calava o rastreador e prendia o chute no cache. A sonda-chute
# depois PROVOU essa hipotese falsa: control_reference a 186 Hz, commandTopic a
# 37 Hz, kick=6.0 em 365 amostras, com o robo parado. Troquei a melhor
# configuracao medida por uma teoria errada.
#
# -60 fica 34 mm a frente do ponto de chute: o robo ainda avanca para dentro da
# bola, mas devagar, sem atravessar de raspao como acontecia com +80.
# O ALVO PROXIMO E O MEIO DA JANELA DE DISPARO.
#
# ARITMETICA, medida no referencial do robo (o que o grSim arbitra):
#     xx = distancia_ao_longo_do_corpo - 73      dispara com 0 <= xx < 31,5
# Logo o CENTRO do robo tem de parar entre 73 e 104,5 mm da bola.
#
# Com o alvo em -60 o pedido era 60 mm - mais perto que a propria placa, e
# impossivel: o casco (90) com a bola (21) trava em 111 mm. Medido: xx=-13,
# fora da janela, e o robo empurrando com o ombro para alcancar um ponto onde
# nao cabe. Resultado do lote: xx na janela em 2 de 14 ciclos armados.
#
# -88 pede o centro a 88 mm, o que da xx = 15 mm: o meio da janela, com folga
# dos dois lados para o atraso do rastreio.
ATRAVESSA_CHUTE = -53.0
LATERAL_CONTORNO = 550.0
# Quanto a frente da bola o portador se oferece quando nao e ele que busca.
PORTADOR_ESPACO = 1500.0
# A que distancia da bola o portador se planta para tapar a linha de chute,
# quando a posse e deles e os outros dois ja estao disputando. 600 mm e fora do
# alcance de contato (111 mm) e perto o bastante para cobrir angulo util.
BLOQUEIO_DIST = 600.0
# Onde a cobertura fica na reta bola->nosso gol: 0 e na bola, 1 e no gol.
# 0,45 a deixa mais perto da bola que do gol, dentro do corredor do chute e com
# tempo de reagir ao rebote.
COBERTURA_FRACAO = 0.45
# Com a bola a menos disto do nosso gol, a cobertura para de cobrir a linha e
# vai NA bola. 2500 mm cobre o nosso terco defensivo (campo de 9000).
COBERTURA_MATA = 2500.0
# Nosso terco defensivo: campo de 9000, gol em -4500, entao 3000 mm de raio
# cobre o terco. Dentro disto a prioridade e AFASTAR a bola.
TERCO_DEFENSIVO = 3000.0
RAIO_MIRA = 400.0
# Portao fisico: a face do chutador tem 80 mm a 73 mm do centro (~29 graus).
# 25 graus de face deixa margem para o rastreio; 20 para a mira.
TOL_FACE = 0.44
TOL_MIRA = 0.35
# Ate quantos graus do eixo de ataque o corpo pode estar e ainda assim chutar.
# 100 graus (1,75 rad) permite o alivio lateral e proibe o chute para tras.
TOL_PARA_TRAS = 1.75
# ARMAR CEDO E MANTER ARMADO ATE O CONTATO.
#
# GEOMETRIA DO GRSIM: a placa do chutador fica a 73 mm do centro do robo e a
# bola so dispara se estiver a no maximo 31,5 mm dela (KickerThickness*2 +
# BallRadius) - ou seja, centro da bola a ate ~104,5 mm do centro do robo.
#
# Armavamos em d < 150 e o unico disparo medido foi em d = 110: 5,5 mm FORA da
# janela. O adversario arma em d < 140 e funciona porque e comandado a ~50 Hz -
# fica armado o tempo todo e pega o instante em que a distancia cruza 104,5. Nos
# rodamos a 10 Hz: entre um ciclo e outro o robo percorre dezenas de milimetros
# e a janela passa sem ninguem olhar.
#
# A correcao nao e mirar melhor a janela, e ficar armado durante toda a
# aproximacao final. O chutador do grSim so dispara quando a geometria permite,
# entao armar cedo nao gasta nada - e a trava de seguranca continua impedindo
# que algo saia na direcao da nossa meta.
ALCANCE_CONTATO = 260.0
# Uma vez armado, so desarma quando a bola se afasta disto - ela saiu, ou
# perdemos a disputa. Mais largo que o alcance de armar, de proposito.
ALCANCE_SOLTA_TRAVA = 420.0
# Dentro disto o robo ja aponta na direcao do chute em vez de olhar para a bola.
RAIO_ORIENTA_CHUTE = 700.0
PRESSAO_RAIO = 400.0
PRESSAO_MIN = 1
LIMITE_Y = 2600.0


def linha_livre(ox, oy, dx_, dy_, inimigos, folga=FOLGA_LINHA):
    """Nenhum adversario a menos de 'folga' do segmento origem -> destino."""
    vx, vy = dx_ - ox, dy_ - oy
    comp2 = vx * vx + vy * vy
    if comp2 <= 1.0:
        return True
    for r in (inimigos or {}).values():
        wx, wy = r.position_x - ox, r.position_y - oy
        t = max(0.0, min(1.0, (wx * vx + wy * vy) / comp2))
        if hypot(wx - t * vx, wy - t * vy) < folga:
            return False
    return True


def alvo_do_chute(ball, gol_ataque, ally_robots, enemy_robots, papeis, estado=None):
    """Para onde o portador deve mandar a bola: o gol, o apoio, ou lugar nenhum.

    POR QUE ISTO PRECISOU EXISTIR
    -----------------------------
    O chute saia sempre na direcao do gol, sem olhar quem estava no caminho.

    MEDIDO nos replays, 19 instantes de chute (bola acima de 2500 mm/s) em 6
    execucoes: em 14 deles - 74% - havia um ADVERSARIO na linha, a menos de 2 m.
    Ou seja, o time recebia a bola e chutava no adversario; ela voltava solta e o
    ciclo recomecava. Era isso que prendia a bola no nosso campo.

    A cobranca de falta ja resolvia isso ha tempos (_folga_do_tiro e
    _ponto_de_mira); o jogo corrido nunca teve equivalente.

    ORDEM DE PREFERENCIA:
      1. o GOL, se a linha estiver limpa;
      2. o APOIO, se a linha ate ele estiver limpa - e o passe que o Felipe
         notou faltar mesmo com o apoio fazendo o pivo certo;
      3. NADA. Nao chutar e melhor que chutar no adversario: a bola volta solta
         e normalmente para eles.
    """
    bx, by = ball.position_x, ball.position_y
    if linha_livre(bx, by, gol_ataque.x, gol_ataque.y, enemy_robots):
        if estado is not None:
            estado.pop("alvo_passe", None)
        return gol_ataque.x, gol_ataque.y, "gol"

    # PASSE COM ALVO CONGELADO.
    #
    # BUG QUE ISTO CORRIGE, e era o que impedia a angulacao: o alvo do passe e
    # um COMPANHEIRO, e companheiro anda. A cada ciclo a linha bola->apoio
    # girava um pouco, o ponto de encaixe (atras da bola NAQUELA linha) pulava
    # junto, e o portador perseguia um ponto que nao esperava. Ele nunca ficava
    # alinhado, entao nunca passava para a fase de avanco - e nunca angulava.
    #
    # MEDIDO: a janela do chutador ficou aberta 1272 quadros com o chute armado
    # em ZERO deles. A geometria permitia e nos nao estavamos prontos.
    #
    # E exatamente o mesmo defeito que a cobranca de falta teve, e a mesma
    # solucao: congelar a posicao do receptor no instante da decisao. A linha
    # continua sendo re-verificada contra a posicao ATUAL dele - se ele se mudar
    # e a linha fechar, a jogada escolhe outra coisa.
    if estado is not None:
        congelado = estado.get("alvo_passe")
        if congelado is not None:
            rid_c = congelado[2]
            atual = ally_robots.get(rid_c)
            if (atual is not None and papeis.get(rid_c) == PAPEL_APOIO
                    and linha_livre(bx, by, atual.position_x, atual.position_y,
                                    enemy_robots)):
                return congelado[0], congelado[1], "passe"
            estado.pop("alvo_passe", None)

    for rid, papel in papeis.items():
        if papel != PAPEL_APOIO or rid not in ally_robots:
            continue
        a = ally_robots[rid]
        if linha_livre(bx, by, a.position_x, a.position_y, enemy_robots):
            if estado is not None:
                estado["alvo_passe"] = (a.position_x, a.position_y, rid)
            return a.position_x, a.position_y, "passe"
    return None, None, "bloqueado"


# Geometria do apoio: a FRENTE da bola e ABERTO, fora do corredor do portador.
#
# 1800/1600, e nao 1200/900: com 1200 a frente e 900 de lado ele ficava DENTRO
# do corredor por onde o portador precisa empurrar, e o chute caiu de 5525 para
# 1536 mm/s. Medido duas vezes, revertido duas vezes.
# Recuo do apoio quando a via esta obstruida: atras da bola, onde a linha
# costuma estar limpa porque a marcacao vem de frente.
APOIO_RECUO = 900.0
APOIO_AVANCO = 1800.0
APOIO_ABERTURA = 1600.0


def alvo_do_papel(papel, situacao, rid, ally_robots, ball, gol_ataque, nosso_gol,
                  ordem=0, alvo_chute=None, inimigos=None, bloqueado=False):
    """Para onde cada robo vai, por (situacao, papel). Devolve (x, y, chuta).

    A matriz esta em documentacao/estrategia/CASOS_DE_JOGO.md.
    """
    bx, by = ball.position_x, ball.position_y
    r = ally_robots[rid]
    rx, ry = r.position_x, r.position_y

    # versor do ATAQUE. Quando ha um alvo de chute escolhido (gol livre ou
    # apoio), o empurrao vai NA DIRECAO DELE - senao o robo empurra para o gol e
    # o chute sai para outro lado, que e a incoerencia que deixava a bola bater
    # no adversario.
    if alvo_chute is not None:
        dax, day = alvo_chute[0] - bx, alvo_chute[1] - by
    else:
        dax, day = gol_ataque.x - bx, gol_ataque.y - by
    na = hypot(dax, day) or 1.0
    ux, uy = dax / na, day / na
    px, py = -uy, ux                      # perpendicular

    def _no_campo(x, y):
        return max(-4300.0, min(4300.0, x)), max(-2800.0, min(2800.0, y))

    # ---------------------------------------------------------------- APOIO
    if papel == PAPEL_APOIO:
        if situacao == SITUACAO_SOLTA:
            # vai para onde a bola VAI PARAR, nao para onde ela esta: chegar
            # depois dela nao adianta.
            fx, fy = onde_a_bola_vai(ball, 1.0)
            return _no_campo(fx, fy) + (False,)
        # O APOIO E O BUSCADOR: e ELE que vai a bola.
        #
        # Inverteu em relacao ao que havia aqui. Antes o mais proximo da bola
        # virava portador e ia buscar, entao toda recuperacao custava a nossa
        # posicao ofensiva. Agora o portador fica adiantado e quem busca e este,
        # que ja estava atras - o pedido do Felipe: "liberar o portador para
        # atacar o espaco".
        #
        # Com a bola NOSSA ele deixa de buscar e vira segundo atacante, a frente
        # e aberto, para receber.
        if situacao == SITUACAO_NOSSA:
            lado = 1.0 if (ordem % 2 == 0) else -1.0
            return _no_campo(bx + ux * APOIO_AVANCO + px * APOIO_ABERTURA * lado,
                             by + uy * APOIO_AVANCO + py * APOIO_ABERTURA * lado) + (False,)

        # SOLTA, DELES, DISPUTA: vai na bola. Em movimento, ao ponto de
        # interceptacao, e sempre ALEM dele - frear em cima da bola foi o que
        # travou o portador antes (medido: um toque em 25 s, com o campo livre).
        # CONTORNAR ATE O LADO CERTO, DE FORMA CONTINUA.
        #
        # Medido no portao, com 50 ciclos de posse nossa: a bola estava na face
        # do chutador em 94 de 96 ciclos, mas o corpo apontava para o NOSSO gol
        # em 82 deles. Ou seja, chegamos na bola pelo lado errado - vindo de
        # cima, ficando entre a bola e o gol deles. Chutar dali seria mandar
        # para o nosso gol, entao a trava barra e so sai trombada.
        #
        # Como a bola sai na direcao robo->bola, o que decide o chute e o LADO
        # por onde se chega. O alvo entao desliza continuamente:
        #   desalinhado  -> um ponto ATRAS da bola (contorna)
        #   alinhado     -> um ponto ALEM dela (atravessa e chuta)
        #
        # 'alinhado' aqui e o angulo entre a minha chegada e a direcao que a
        # bola deve tomar. Sem degrau: quando a chegada melhora, o alvo avanca
        # sozinho. A versao com fronteira fixa fazia o robo orbitar em cima do
        # limite - ver AVANCO_SOLTA.
        _dir_alvo = alvo_chute if alvo_chute is not None else (
            bx + ux * 1000.0, by + uy * 1000.0)
        _a_alvo = atan2(_dir_alvo[1] - by, _dir_alvo[0] - bx)
        _a_cheg = atan2(by - ry, bx - rx)
        _t = 1.0 - min(abs(_norm_ang(_a_cheg - _a_alvo)) / pi, 1.0)   # 1 = perfeito
        _ux_a, _uy_a = cos(_a_alvo), sin(_a_alvo)

        avanco_base = AVANCO_SOLTA if situacao == SITUACAO_SOLTA else AVANCO_PORTADOR

        # PERTO, MIRA O PONTO DE CHUTE; LONGE, MIRA ALEM DA BOLA.
        #
        # GEOMETRIA: o casco tem raio 90 mm e a bola 21, entao tocando o casco
        # redondo a distancia trava em 111 mm e o chutador - que fica a 73 mm do
        # centro - nunca alcanca a bola. Para a bola entrar na placa, o CENTRO
        # do robo precisa ficar a ~94 mm dela (73 + 21), com a face plana
        # apresentada. Medido nos replays: chegamos a 95 e 99 mm, dentro da
        # janela, mas em UM unico quadro de cada partida - passamos de raspao
        # em vez de estacionar ali.
        #
        # Mirar 500-1600 mm ALEM da bola faz o robo chegar rapido e bater com o
        # ombro: a bola escapa antes do disparo. Mirar o PONTO DE CHUTE faz ele
        # desacelerar apresentando a face.
        #
        # Os dois regimes num alvo continuo: longe vale o avanco (velocidade),
        # perto vale o ponto de chute (precisao), interpolando pela distancia.
        # distancia do robo a bola neste ramo (o 'd_bola' do portador esta
        # noutro escopo - foi o que quebrou na primeira versao)
        _d_ate_bola = hypot(rx - bx, ry - by)
        _c = min(max((_d_ate_bola - PONTO_CHUTE) / (RAIO_ENCAIXE - PONTO_CHUTE),
                     0.0), 1.0)
        # NUNCA "CHEGAR": o comando de chute so viaja com o robo em movimento.
        #
        # O kick vai dentro do TeamCommand, e o control.py so publica esse
        # TeamCommand se 'control_references' nao estiver vazio (control.py:120)
        # - ou seja, so enquanto o RASTREADOR emite referencia, que e so
        # enquanto o robo esta indo a algum lugar.
        #
        # Mirar exatamente o ponto de chute fazia o robo CHEGAR: o rastreador se
        # calava, o commandTopic parava, e o chute ficava preso no kick_cache,
        # armado, sem nunca ser enviado. Quanto melhor o encaixe, mais garantido
        # que o chute nao saia - medido: 'arma=True' 34 a 110 vezes por lote e
        # ZERO disparos, com o robo a 92-96 mm da bola (dentro da janela).
        #
        # O alvo proximo passa a ser ALEM da bola, nao o ponto de chute. O robo
        # atravessa devagar - o trajeto de ~175 mm mantem o rastreador vivo - e
        # passa pela janela de disparo no caminho, com o chute ja armado.
        _off_alinhado = ATRAVESSA_CHUTE + (avanco_base - ATRAVESSA_CHUTE) * _c
        # PERTO DA BOLA, COMPROMETE-SE COM A JANELA.
        #
        # O termo de contorno dependia so do alinhamento. Com o robo a 90 mm da
        # bola e alinhamento imperfeito (_t = 0,6), o alvo virava
        #     -700*0,4 + (-88)*0,6 = -333 mm
        # ou seja: ele era mandado para 333 mm ATRAS da bola estando colado
        # nela, e recuava. Media disso: 5 ciclos armados por replay - ele quase
        # nunca chegava a receber o alvo da janela.
        #
        # O lado por onde contornar ja foi escolhido la longe. Na chegada nao ha
        # mais o que decidir: o alvo e o meio da janela de disparo, e ponto.
        _off_longe = -RECUO_CONTORNO * (1.0 - _t) + _off_alinhado * _t
        _off = _c * _off_longe + (1.0 - _c) * ATRAVESSA_CHUTE
        # desvio lateral pelo lado em que o robo ja esta, para contornar em arco
        _cruz = _ux_a * (ry - by) - _uy_a * (rx - bx)
        _lado = 1.0 if _cruz >= 0 else -1.0
        # O DESVIO LATERAL TEM DE SUMIR AO CHEGAR.
        #
        # Ele dependia so do alinhamento: perto da bola, com alinhamento ainda
        # imperfeito, seguia empurrando o alvo 550 mm para o lado - e o robo
        # passava AO LADO da bola em vez de atraves dela.
        #
        # Medido no referencial do robo (o que o grSim arbitra, robot.cpp:120):
        # em 15 ciclos armados, 'dispara' foi False nos 15. O desvio lateral yy
        # foi de -73 a -260 mm contra um limite de 40, e SEMPRE do mesmo lado,
        # crescendo - o robo derivando enquanto a trava o mantinha armado. A
        # bola ficava no ombro, nunca na placa. E o "esta com a bola e nao faz
        # nada" que se ve no replay.
        #
        # Multiplicando pelo mesmo fator de distancia do avanco, o contorno vale
        # longe (onde ele serve para escolher o lado) e zera na chegada, onde o
        # robo precisa estar EM CIMA da linha de tiro.
        _lat = LATERAL_CONTORNO * (1.0 - _t) * _lado * _c + VIES_LATERAL
        return _no_campo(bx + _ux_a * _off - _uy_a * _lat,
                         by + _uy_a * _off + _ux_a * _lat) + (True,)

        avanco = AVANCO_SOLTA if situacao == SITUACAO_SOLTA else AVANCO_PORTADOR
        if bola_ja_saiu(ball):
            fx, fy = onde_a_bola_vai(ball, 0.5)
            dxp, dyp = fx - rx, fy - ry
            n_p = hypot(dxp, dyp) or 1.0
            return _no_campo(fx + (dxp / n_p) * avanco,
                             fy + (dyp / n_p) * avanco) + (True,)
        dxp, dyp = bx - rx, by - ry
        n_p = hypot(dxp, dyp) or 1.0
        return _no_campo(bx + (dxp / n_p) * avanco,
                         by + (dyp / n_p) * avanco) + (True,)

    # ------------------------------------------------------------ COBERTURA
    if papel == PAPEL_COBERTURA:
        # entre a bola e o NOSSO gol, espalhado para nao empilhar.
        # Na bola solta ela MANTEM a posicao - correr atras de bola solta e
        # deixar o contra-ataque aberto.
        # DELES e DISPUTA: a COBERTURA e o SEGUNDO a ir a bola.
        #
        # "Quando for deles vao dois". Ela entra pelo lado oposto ao do
        # buscador: sem dribbler, a direcao do empurrao e dada pela POSICAO de
        # quem encosta, entao dois angulos de ataque dao duas saidas possiveis
        # em vez de uma. O portador segue adiantado, esperando a recuperacao.
        if situacao in (SITUACAO_DELES, SITUACAO_DISPUTA):
            lado_c = 1.0 if (ordem % 2 == 0) else -1.0
            return _no_campo(bx + px * 240.0 * lado_c + ux * 100.0,
                             by + py * 240.0 * lado_c + uy * 100.0) + (True,)

        # SOBRE A LINHA DE TIRO, nao ao lado dela.
        #
        # O ponto base ja era o meio do caminho entre a bola e o nosso gol - o
        # lugar certo. Mas o deslocamento para nao empilhar era PERPENDICULAR a
        # essa linha (-uy, +ux), ou seja tirava a cobertura de cima dela de
        # proposito, em 800 mm ou mais. Ela ficava ao lado do corredor por onde
        # o chute passa.
        #
        # Medido nos tres replays: o amarelo chuta do meio-campo a 5300-5400
        # mm/s, a bola percorre 4122, 5437 e 4205 mm em linha reta ate a nossa
        # linha de fundo, atravessa o time inteiro, e o unico que toca nela e o
        # GOLEIRO. Nenhum robo de linha estava na trajetoria.
        #
        # Agora o primeiro fica EM CIMA da reta bola->nosso gol. Quem sobra se
        # espalha AO LONGO dela, mais perto do gol, em vez de para os lados:
        # dois corpos no mesmo corredor cobrem o rebote, dois ao lado nao cobrem
        # nada.
        dgx, dgy = nosso_gol.x - bx, nosso_gol.y - by
        n_g = hypot(dgx, dgy) or 1.0
        lgx, lgy = dgx / n_g, dgy / n_g     # bola -> nosso gol
        # Vale em TODAS as situacoes, inclusive bola solta: antes ela mantinha
        # a posicao na bola solta, o que a deixava fora da linha justamente
        # quando o chute vem.
        # DEFESA QUE MATA A JOGADA, em vez de esperar o chute.
        #
        # Pedido do Felipe: "a cobertura, depois de chegar a linha de fundo, tem
        # que chegar pra matar na bola, para impedir totalmente o chute".
        #
        # Ficar na reta bola->gol e bom contra bola rolando, e inutil contra um
        # chute que cobre o campo em 0,8 s: medimos a bola percorrendo 4122,
        # 5437 e 4205 mm em linha reta ate a nossa linha de fundo, atravessando
        # o time inteiro. Bloquear a 45% da linha nao chega a tempo.
        #
        # Com a bola no NOSSO terco defensivo nao ha o que esperar - a unica
        # defesa que funciona e tirar o espaco de chute encostando nela. Fora
        # dali ela volta a cobrir a linha, que e o certo com a bola longe.
        dist_gol = hypot(bx - nosso_gol.x, by - nosso_gol.y)
        if dist_gol < COBERTURA_MATA and ordem == 0:
            # vai NA bola, nao na linha - e alem dela, para nao chegar freando
            dxm, dym = bx - rx, by - ry
            n_m = hypot(dxm, dym) or 1.0
            return _no_campo(bx + (dxm / n_m) * 400.0,
                             by + (dym / n_m) * 400.0) + (True,)

        recuo_extra = 500.0 * ordem          # o segundo fica mais perto do gol
        alvo_l = n_g * COBERTURA_FRACAO + recuo_extra
        alvo_l = min(alvo_l, n_g - 200.0)    # nunca dentro da propria bola
        return _no_campo(bx + lgx * alvo_l, by + lgy * alvo_l) + (False,)

    # ------------------------------------------------------------- PORTADOR
    #
    # REESCRITO, e a regra e curta de proposito:
    #
    #   1. vai NA BOLA, direto, sempre;
    #   2. pegou? tem companheiro livre a frente? toca nele e avanca para
    #      receber de novo;
    #   3. nao tem? leva a bola para frente ate a linha do gol abrir.
    #
    # O QUE ISTO SUBSTITUI, e por que
    # -------------------------------
    # A versao anterior tinha ponto de encaixe: ele mirava um ponto ATRAS da
    # bola (150-250 mm), esperava ficar alinhado e so entao atravessava. Duas
    # medicoes mataram esse desenho:
    #
    #  - com o alvo sempre a ~200 mm dele (mediana medida: 201 mm), o encaixe
    #    se deslocava junto com a bola a cada ciclo. Ele orbitava a bola sem
    #    nunca alcanca-la: 'arma' fechou 8 vezes em 203 ciclos;
    #  - com o ADVERSARIO PARADO - campo livre, ninguem disputando - o time
    #    tocou na bola UMA vez em 25 s, em dois dos tres replays. Sem oposicao
    #    nenhuma. O defeito nunca foi perder dividida.
    #
    # E a orientacao vinha junto: o portador ficava com 31-33 graus de erro
    # medio para a bola e de COSTAS para ela em ~20% do tempo, enquanto apoio e
    # cobertura ficavam com 2-3 graus. Era a linha de tiro, calculada a partir
    # da posicao da BOLA: com o robo do lado errado, ela aponta para longe
    # dela. O adversario que funciona - o perfil antigo, amarelo - nao tem
    # encaixe nem espera alinhamento, e tocou 62 vezes num replay em que nos
    # tocamos uma.
    proj = (rx - bx) * ux + (ry - by) * uy
    d_bola = hypot(rx - bx, ry - by)

    # PERDEU A DISPUTA? ENTAO TAPA A LINHA DE CHUTE.
    #
    # Pedido do Felipe: "se o nosso portador nao conseguir ganhar a disputa, ele
    # pelo menos bloqueie a linha de chute".
    #
    # Com a bola deles e alguem NOSSO ja disputando (o buscador e a cobertura
    # vao), o portador nao tem o que fazer indo tambem - seria o terceiro na
    # mesma bola. Mas ficar 1500 mm a frente esperando tambem nao serve, porque
    # medimos a bola saindo do pe deles direto para o nosso fundo: 5461 e
    # 5377 mm em linha reta, com UM unico toque na partida inteira.
    #
    # Entre a bola e o NOSSO gol, colado no adversario que a tem, ele tira o
    # angulo. E o mesmo principio da cobertura, aplicado a quem esta perto.
    if situacao == SITUACAO_DELES and inimigos:
        dono = min(inimigos.values(),
                   key=lambda e: hypot(e.position_x - bx, e.position_y - by))
        d_dono = hypot(dono.position_x - bx, dono.position_y - by)
        if d_dono < RAIO_POSSE:
            gx_, gy_ = nosso_gol.x - bx, nosso_gol.y - by
            n_b = hypot(gx_, gy_) or 1.0
            return _no_campo(bx + gx_ / n_b * BLOQUEIO_DIST,
                             by + gy_ / n_b * BLOQUEIO_DIST) + (False,)

    # O PORTADOR NAO VAI MAIS BUSCAR. Ele ataca o espaco e espera a bola.
    #
    # Quem busca e o apoio (buscador); em DELES e DISPUTA a cobertura vai junto.
    # Se o portador tambem descesse, ninguem estaria a frente quando a bola
    # fosse recuperada, e era isso que acontecia: zero quadros alem de x=3500 em
    # todas as execucoes, em todos os lotes desta fase.
    #
    # Com a bola longe e nao nossa, ele se oferece: adiante da bola, no meio.
    # SE ELE ESTA EM CIMA DA BOLA, NAO LARGA.
    #
    # Observado no replay: "o portador recebe a bola, toca nela e nao faz nada,
    # ou se afasta da bola". O ramo abaixo manda o portador se oferecer 1500 mm
    # a frente quando a situacao e DELES ou SOLTA. Mas a situacao e geometrica e
    # oscila: com o robo a 260 mm da bola e o raio de posse em 250, um quadro de
    # rastreio ruim vira SOLTA e ele ABANDONA a bola que acabou de receber, indo
    # embora para a frente.
    #
    # Quem esta a menos de ENGAJA_RAIO e e o nosso mais proximo dela, disputa -
    # nao importa como a situacao esta rotulada naquele ciclo.
    _meu = all(d_bola <= hypot(o.position_x - bx, o.position_y - by) + 1.0
               for rid_o, o in ally_robots.items()
               if rid_o != 0 and rid_o != rid)
    if d_bola < ENGAJA_RAIO and _meu:
        pass          # cai no tratamento normal de bola: atravessa e chuta
    elif situacao in (SITUACAO_DELES, SITUACAO_SOLTA) and d_bola > RAIO_POSSE_PORTADOR:
        return _no_campo(bx + ux * PORTADOR_ESPACO, by + uy * PORTADOR_ESPACO * 0.3) + (False,)

    # UM ALVO SO: o ponto logo ALEM da bola, na direcao em que ela deve ir.
    #
    # Dirigir para la significa atravessar a bola - longe ou perto, a instrucao
    # e a mesma. Nao existe fronteira de posse, e e de proposito.
    #
    # BUG QUE ISTO CORRIGE: a primeira versao desta reescrita tinha a fronteira
    # em 200 mm - fora dela o alvo era a bola, dentro dela o alvo saltava para
    # 500 mm alem. O robo cruzava a fronteira, o alvo pulava 500 mm, ele
    # recuava, cruzava de volta, e ficava oscilando em cima do limite sem se
    # comprometer. Medido com o adversario PARADO: ele encostava na bola uma
    # vez, por volta de 4 s, e nunca mais - 'arma' nao fechou nenhuma vez em
    # 166 ciclos, e a distancia ao alvo nunca desceu de 202 mm (mediana 337).
    #
    # Alvo continuo, sem degrau: ele chega, atravessa e segue.
    if bola_ja_saiu(ball):
        # em movimento, persegue onde ela VAI estar - nunca fica parado.
        fx, fy = onde_a_bola_vai(ball, 0.5)
        dxp, dyp = fx - rx, fy - ry
        n_p = hypot(dxp, dyp) or 1.0
        return _no_campo(fx + (dxp / n_p) * AVANCO_PORTADOR,
                         fy + (dyp / n_p) * AVANCO_PORTADOR) + (True,)

    # Parada: a direcao e a do ALVO DO CHUTE quando existe (companheiro livre a
    # frente, ou o gol), senao a do ataque. Atravessar nessa direcao e ao mesmo
    # tempo o "toca nele" e o "leva para frente ate a linha abrir".
    if alvo_chute is not None:
        avx, avy = alvo_chute[0] - bx, alvo_chute[1] - by
        n_av = hypot(avx, avy) or 1.0
        dirx, diry = avx / n_av, avy / n_av
    else:
        dirx, diry = ux, uy
    return _no_campo(bx + dirx * AVANCO_PORTADOR,
                     by + diry * AVANCO_PORTADOR) + (True,)


class CenterGoal:
    # 2250 -> 4500: o gol da Division B fica em x = +-4500, nao +-2250.
    #
    # 2250 e a meia-largura de um campo SSL-EL (4500 x 3000). Este projeto roda
    # em Division B: 9000 x 6000, confirmado pelas regras oficiais (sslrules.pdf
    # secao 2.1.1) e pelo proprio /game_state, que reporta campo=9000mm.
    #
    # O QUE O VALOR ERRADO CAUSAVA, e nao e sutil: o goleiro se posicionava
    # 2250 mm A FRENTE da propria meta - ou seja, abandonava o gol e parava
    # perto do meio-campo - e os atacantes miravam um ponto vazio no meio do
    # campo adversario. Em jogo aberto o time inteiro converge para o centro.
    #
    # A mesma constante ja existia errada em tatics/freekick.py e foi corrigida
    # la ha tempos; kickoff.py, stop.py e running.py ficaram para tras (o
    # HANDOVER §6.2 registra as tres como "nao corrigidas"). Esta e a correcao
    # que faltava.
    GOAL_POSITIVE = Vector2D(4500.0, 0.0)
    GOAL_NEGATIVE = Vector2D(-4500.0, 0.0)


def montar_comandos(tt):
    """Monta o comando de cada robo a partir de (situacao, papel).

    As duas taticas - Atack e Defense - passam por aqui. Antes cada uma
    tinha a sua propria nocao de "o eleito e os outros", e a defesa sequer
    rodava (DefenseAction instanciava Atack). Uma logica so evita que elas
    divirjam de novo.
    """
    situacao = situacao_de_jogo(tt.ally_robots, tt.enemy_robots, tt.ball)
    papeis = distribuir_papeis(tt.ally_robots, tt.ball, situacao,
                               getattr(tt, "estado", None),
                               sentido=(-1.0 if tt.on_positive_half
                                        else 1.0))
    nosso_gol = (tt.goal_center.GOAL_POSITIVE if tt.on_positive_half
                 else tt.goal_center.GOAL_NEGATIVE)
    gol_ataque = (tt.goal_center.GOAL_NEGATIVE if tt.on_positive_half
                  else tt.goal_center.GOAL_POSITIVE)

    if os.environ.get("DIAG_JOGO"):
        print("[JG] situacao=%s papeis=%s" % (situacao, papeis), flush=True)
        print("[JG] lado=%s gol_ataque=%.0f nosso_gol=%.0f bola=%.0f,%.0f" %
              (tt.on_positive_half, gol_ataque.x, nosso_gol.x,
               tt.ball.position_x, tt.ball.position_y), flush=True)

    # PARA ONDE A BOLA DEVE IR neste ciclo: gol, apoio, ou lugar nenhum.
    ax, ay, tipo_alvo = alvo_do_chute(tt.ball, gol_ataque,
                                      tt.ally_robots, tt.enemy_robots,
                                      papeis, getattr(tt, "estado", None))
    alvo_chute = None if ax is None else (ax, ay)

    # A MIRA NAO PODE TROCAR A TODO MOMENTO.
    #
    # A orientacao de quem esta perto da bola e a direcao bola->alvo_chute. Se o
    # alvo troca, a linha de tiro GIRA e o robo gira atras dela.
    #
    # MEDIDO no replay, giro perto da bola:
    #     nossos    p50 82-84 graus/s   p90 155-175
    #     amarelos  p50 36-40 graus/s   p90  89-103
    # Mais que o DOBRO. E eles aparecem perto da bola em 1230-1989 quadros por
    # partida, nos em 95: eles ficam la, nos passamos girando.
    #
    # A orientacao deles e bola->gol, que muda devagar porque so a bola se move.
    # A nossa trocava de TIPO a cada 10,9 ciclos (23 trocas em 250), e cada
    # troca gol<->passe gira a mira dezenas de graus.
    #
    # Mesma trava que estabilizou os papeis (133 trocas -> 1 em 250 ciclos):
    # o tipo escolhido vale por CICLOS_MIN_MIRA ciclos, desde que continue
    # valido. Trocar so quando o anterior deixa de existir.
    _est = getattr(tt, "estado", None)
    if _est is not None:
        _ant_t = _est.get("mira_tipo")
        _ant_p = _est.get("mira_ponto")
        _n = _est.get("mira_ciclos", 0)
        if (_ant_t is not None and _n < CICLOS_MIN_MIRA
                and tipo_alvo not in (None, "bloqueado")
                and _ant_t not in (None, "bloqueado")):
            tipo_alvo, alvo_chute = _ant_t, _ant_p
            _est["mira_ciclos"] = _n + 1
        else:
            _est["mira_tipo"] = tipo_alvo
            _est["mira_ponto"] = alvo_chute
            _est["mira_ciclos"] = 0

    # SAIDA DE BOLA: no nosso terco, AFASTAR vem antes de construir.
    #
    # Medido nos replays: o adversario chuta do meio e a bola percorre 3100 a
    # 5400 mm ate o nosso fundo, e FICA la - num deles, 267 toques com a bola
    # parada em x=-4150. Dali a linha ate o gol deles tem 8,6 m e atravessa o
    # campo inteiro: sempre ha alguem nela, entao 'alvo_chute' vira 'bloqueado'
    # e ninguem faz nada. Nao e o criterio que erra, e a geometria.
    #
    # No terco defensivo a prioridade se inverte: tirar a bola vale mais que
    # procurar jogada. Preferimos o passe quando ele existe - e progressao - e
    # caimos no afastamento quando nao existe, em vez de ficar sem alvo.
    #
    # DIVISION B, Aimless Kick: se a bola cruzar o meio e sair pela linha de
    # fundo DELES sem tocar em ninguem, a falta e deles. Por isso o afastamento
    # mira a meia altura do campo e para a LATERAL, nao o fundo - queremos que
    # ela pare em campo.
    _dist_nosso_gol = hypot(tt.ball.position_x - nosso_gol.x,
                            tt.ball.position_y - nosso_gol.y)
    if _dist_nosso_gol < TERCO_DEFENSIVO and tipo_alvo in (None, "bloqueado"):
        _sent = 1.0 if gol_ataque.x >= 0 else -1.0
        _ly = LIMITE_Y if tt.ball.position_y >= 0 else -LIMITE_Y
        _cand = [(tt.ball.position_x + 1800.0 * _sent, _ly * 0.7),
                 (tt.ball.position_x + 2200.0 * _sent, 0.0),
                 (tt.ball.position_x + 1800.0 * _sent, -_ly * 0.7)]
        # A MELHOR DIRECAO DISPONIVEL, nao a perfeita.
        #
        # A primeira versao exigia linha LIVRE para afastar, e disparou 2 vezes
        # em 170 - 'bloqueado' nas outras 168. Foi erro de conceito: afastar e
        # justamente o que se faz quando nada esta livre. Com a bola na nossa
        # area e eles em cima, nenhuma direcao esta limpa, e exigir limpeza
        # equivale a nao ter saida nenhuma.
        #
        # Escolhemos a direcao com MAIOR folga ate o adversario mais proximo da
        # linha - o mesmo principio da varredura do ITAndroids, que mira na
        # bissetriz do maior intervalo livre em vez de exigir intervalo perfeito.
        _melhor, _melhor_folga = None, -1.0
        for _cx, _cy in _cand:
            _dx, _dy = _cx - tt.ball.position_x, _cy - tt.ball.position_y
            _n = hypot(_dx, _dy) or 1.0
            _ux_c, _uy_c = _dx / _n, _dy / _n
            _folga = 9999.0
            for _e in (tt.enemy_robots or {}).values():
                _px = _e.position_x - tt.ball.position_x
                _py = _e.position_y - tt.ball.position_y
                _proj = _px * _ux_c + _py * _uy_c
                if _proj <= 0 or _proj > _n:
                    continue                      # atras de nos ou alem do alvo
                _lat = abs(-_px * _uy_c + _py * _ux_c)
                _folga = min(_folga, _lat)
            if _folga > _melhor_folga:
                _melhor, _melhor_folga = (_cx, _cy), _folga
        if _melhor is not None:
            alvo_chute = _melhor
            tipo_alvo = "saida"
        if os.environ.get("DIAG_JOGO"):
            print("[JG] SAIDA d_gol=%.0f -> %s" % (_dist_nosso_gol, tipo_alvo),
                  flush=True)

    # ALIVIO: PRENSADO E SEM LINHA, JOGA NA LATERAL.
    #
    # Com o adversario em cima, 'alvo_do_chute' devolve None - gol e apoio
    # bloqueados - e o portador fica posicionando para sempre. A bola nunca
    # abre, porque quem prensa nao tem motivo para sair. Foi isso que travou o
    # jogo contra o perfil antigo: DISPUTA em 87% do tempo, 100% no nosso campo.
    #
    # Sem saida boa, a saida e a lateral: tirar a bola da prensa vale mais que
    # manter posse dentro dela. A reposicao fica com eles, mas longe do nosso
    # gol e com o campo aberto de novo.
    if alvo_chute is None and tt.enemy_robots:
        rid_p = next((k for k, v in papeis.items() if v == PAPEL_PORTADOR), None)
        if rid_p is not None and rid_p in tt.ally_robots:
            p_ = tt.ally_robots[rid_p]
            perto = sum(1 for e in tt.enemy_robots.values()
                        if hypot(e.position_x - p_.position_x,
                                 e.position_y - p_.position_y) < PRESSAO_RAIO)
            if perto >= PRESSAO_MIN:
                lat_y = LIMITE_Y if tt.ball.position_y >= 0 else -LIMITE_Y
                avanco = 800.0 if gol_ataque.x >= 0 else -800.0
                alvo_chute = (tt.ball.position_x + avanco, lat_y)
                tipo_alvo = "alivio"
                if os.environ.get("DIAG_JOGO"):
                    print("[JG] ALIVIO perto=%d -> %.0f,%.0f"
                          % (perto, alvo_chute[0], alvo_chute[1]), flush=True)
    if os.environ.get("DIAG_JOGO"):
        print("[JG] alvo_chute=%s inimigos=%d situacao=%s" %
              (tipo_alvo, len(tt.enemy_robots or {}), situacao), flush=True)

    comandos = []
    ordem = {PAPEL_APOIO: 0, PAPEL_COBERTURA: 0}
    for rid in sorted(tt.ally_robots):
        if rid == 0:
            continue
        papel = papeis.get(rid, PAPEL_COBERTURA)
        o = ordem.get(papel, 0)
        alvo_x, alvo_y, chuta = alvo_do_papel(
            papel, situacao, rid, tt.ally_robots, tt.ball,
            gol_ataque, nosso_gol, ordem=o, alvo_chute=alvo_chute,
            inimigos=tt.enemy_robots, bloqueado=(tipo_alvo == "bloqueado"))
        if papel in ordem:
            ordem[papel] += 1

        # O CORPO APONTA PARA O ATAQUE quando o robo pode tocar na bola.
        #
        # O tiro sai no eixo do corpo (grSim robot.cpp:157). Quem ganha a
        # bola virado para tras a devolve para eles - foi pedido
        # explicitamente que na disputa o corpo aponte para frente.
        _r0 = tt.ally_robots[rid]
        _d0 = hypot(_r0.position_x - tt.ball.position_x,
                    _r0.position_y - tt.ball.position_y)
        # TODOS OLHAM PARA A BOLA. A CHEGADA E QUE MIRA O CHUTE.
        #
        # SEM DRIBBLER NAO SE GIRA COM A BOLA. Mandar quem tem a bola assumir a
        # linha de tiro tira a bola da face do chutador; olhar para a bola
        # desalinha a mira. Medido no portao: 'mirado' FALSO em 14 de 14 ciclos
        # com a bola, e 'na_face' falso em 12 deles - as duas metades da
        # condicao nunca fechavam juntas, e por isso o time NUNCA chutou.
        #
        # O tiro sai no eixo do corpo (grSim robot.cpp:157) e o corpo aponta
        # para a bola: ela sai na direcao em que o robo VEIO. Nao ha o que
        # mirar, ha que CHEGAR PELO LADO CERTO - e o alvo do buscador ja
        # atravessa a bola na direcao do alvo do chute, entao a aproximacao
        # correta produz o chute correto sozinha.
        _r_o = tt.ally_robots[rid]
        _dperto = hypot(_r_o.position_x - tt.ball.position_x,
                        _r_o.position_y - tt.ball.position_y)
        # A RECEITA DO ADVERSARIO, QUE FUNCIONA.
        #
        # O perfil amarelo - a nossa estrategia antiga - chuta em toda partida,
        # e faz duas coisas que nos nao faziamos:
        #   1. o corpo aponta SEMPRE na direcao em que a bola deve sair, nunca
        #      para a bola;
        #   2. arma o chute so por proximidade (d < 140 mm), sem conferir face
        #      nem mira.
        #
        # Nos tinhamos o oposto: corpo na bola e um portao de duas condicoes que
        # nunca fechavam juntas - medido, 'mirado' falso em 96 de 96 ciclos com
        # a bola. Apontar para a bola garante o contato e impede o disparo util;
        # apontar para o alvo dispara certo quando o contato acontece.
        #
        # Quem esta indo a bola usa a direcao do CHUTE. Os outros seguem olhando
        # para ela, que e o certo para receber e para cobrir.
        if _dperto < RAIO_ORIENTA_CHUTE:
            _mira = alvo_chute or (gol_ataque.x, gol_ataque.y)
            ang = atan2(_mira[1] - tt.ball.position_y,
                        _mira[0] - tt.ball.position_x)
        else:
            ang = atan2(tt.ball.position_y - _r_o.position_y,
                        tt.ball.position_x - _r_o.position_x)

        # Quem esta efetivamente com a bola: o nosso mais proximo dentro do raio
        # de posse. Serve so para o diagnostico - a decisao de chutar e do
        # portao fisico, nao do papel nem deste rotulo.
        _com_a_bola = (_d0 < RAIO_POSSE_PORTADOR and rid != 0 and all(
            _d0 <= hypot(o.position_x - tt.ball.position_x,
                         o.position_y - tt.ball.position_y) + 1.0
            for rid_o, o in tt.ally_robots.items()
            if rid_o != 0 and rid_o != rid))

        if os.environ.get("DIAG_JOGO"):
            _rr = tt.ally_robots[rid]
            print("[JG] r%d papel=%s pos=%.0f,%.0f alvo=%.0f,%.0f d=%.0f"
                  % (rid, papel, _rr.position_x, _rr.position_y, alvo_x, alvo_y,
                     hypot(alvo_x - _rr.position_x, alvo_y - _rr.position_y)),
                  flush=True)
        cmd = tt.skills_factory.move_with_angle(
            robot_id=rid, target_x=alvo_x, target_y=alvo_y,
            vel_x=0.0, vel_y=0.0, angle=ang,
        )
        r = tt.ally_robots[rid]
        d_bola = hypot(r.position_x - tt.ball.position_x,
                       r.position_y - tt.ball.position_y)
        # NAO ARMA COM A LINHA BLOQUEADA.
        #
        # Medido: 74% dos chutes tinham um adversario na linha, a menos de
        # 2 m. Chutar nele devolve a bola solta, quase sempre para eles.
        # Sem alvo, o portador continua posicionando ate abrir.
        #
        # A FORCA depende do alvo: no gol vai o maximo (o atrito do grSim e
        # abrupto - 3 m/s percorre 1633 mm, 6 m/s percorre 4220); num PASSE,
        # forca de gol atravessa o companheiro. 2,5 m/s cobre ~1,5 m, que e
        # a distancia tipica do apoio.
        # passe curto e 2,5 (forca de gol atravessa o receptor); saida e
        # media, para a bola PARAR no campo deles e nao sair pela linha de
        # fundo - Aimless Kick, Division B.
        forca = (2.5 if tipo_alvo == "passe"
                 else 4.5 if tipo_alvo == "saida"
                 else FORCA_CHUTE)
        # ARMAR PELO CORPO, NAO PELA POSICAO.
        #
        # O portao era 'chuta', que exige o robo ATRAS da bola e a menos de
        # 120 mm do eixo de tiro - alinhamento de POSICAO. Sob pressao ele nunca
        # fecha: o adversario encosta antes. Medimos 186 de 250 ciclos em
        # alivio, ou seja prensados, e nesses o chute simplesmente nao saia,
        # mesmo com o apoio livre para receber o passe.
        #
        # Mas o grSim nao dispara pela posicao: dispara no eixo do CORPO
        # (robot.cpp:157), desde que a bola esteja na face do chutador. O
        # criterio fisico e esse - bola na face, corpo apontado para o alvo. Um
        # robo que chega de lado ja girado pode chutar certo, e o portao antigo
        # o proibia.
        if os.environ.get("DIAG_JOGO") and _com_a_bola:
            print("[JG] COM_A_BOLA r%d papel=%s d=%.0f alvo=%s"
                  % (rid, papel, d_bola, tipo_alvo), flush=True)
        if alvo_chute is None or d_bola >= FORCA_CHUTE_ALCANCE:
            arma = False
        else:
            ang_alvo = atan2(alvo_chute[1] - tt.ball.position_y,
                             alvo_chute[0] - tt.ball.position_x)
            ang_bola = atan2(tt.ball.position_y - r.position_y,
                             tt.ball.position_x - r.position_x)
            # ARMA POR PROXIMIDADE, como o adversario faz.
            #
            # O portao anterior exigia bola na face E corpo alinhado com o alvo.
            # Sem dribbler as duas nunca fecham juntas: girar para o alvo tira a
            # bola da face, olhar para a bola desalinha a mira. Medido: 'mirado'
            # falso em 96 de 96 ciclos COM a bola, e o time nunca chutou em
            # nenhuma partida desta fase.
            #
            # Com o corpo ja apontado na direcao do chute (ver a orientacao
            # acima), o contato basta: o tiro sai no eixo do corpo, que e o eixo
            # certo. E o que o perfil amarelo faz, e ele chuta em toda partida.
            #
            # A UNICA trava que fica e a de seguranca: nada sai na direcao da
            # nossa meta, exceto passe - um chute forte para o nosso campo
            # entrega a bola com velocidade onde eles chutam.
            sentido_x = -1.0 if tt.on_positive_half else 1.0
            ang_saida = atan2(alvo_chute[1] - tt.ball.position_y,
                              alvo_chute[0] - tt.ball.position_x)
            para_frente = abs(_norm_ang(ang_saida - (0.0 if sentido_x > 0
                                                     else pi))) < TOL_PARA_TRAS
            # TRAVA: UMA VEZ ARMADO, SEGUE ARMADO ATE A BOLA SAIR.
            #
            # Esta licao ja estava paga na fase de bola parada, e eu nao a
            # trouxe. O comentario de _chute_deve_estar_armado (freekick.py)
            # registra a medicao:
            #
            #   "a janela do chutador abriu em 4 de 6 execucoes, ficou aberta
            #    ~30 quadros (0,5 s) em duas delas, e o disparo nao saiu - com a
            #    estrategia tendo pedido chute em algum outro momento. Pedido e
            #    janela simplesmente nao se encontravam."
            #
            # A causa e o atraso da orientacao da visao: 4,7 graus com o filtro
            # corrigido, 17-23 sem ele - o que vale 8 a 42 mm de desvio lateral
            # so de atraso. Recalcular a condicao a cada ciclo faz o chute
            # PISCAR durante a aproximacao, e acertar o quadro do contato vira
            # sorte. A 10 Hz, com a janela durando poucos quadros, a sorte nao
            # acontece: medimos 73 pedidos de chute e ZERO disparos, com o
            # comando comprovadamente chegando ao grSim (sonda-chute: kick=6.0
            # em 365 amostras do commandTopic).
            #
            # Quem arbitra continua sendo o grSim, que so dispara com a bola na
            # placa (robot.cpp:128). Manter armado nao cria chute torto - a
            # direcao segue protegida pela trava de seguranca abaixo; so deixa
            # de perder o chute certo.
            # A BOLA PRECISA ESTAR NA FRENTE DO CHUTADOR.
            #
            # Sem isto o criterio e simetrico: arma com a bola encostada nas
            # COSTAS do robo. E o que estava acontecendo - medido, a mediana do
            # xx com o chute armado era -244 mm, com 76 de 83 ciclos negativos.
            # O robo ultrapassa a bola, fica a frente dela olhando para longe, e
            # a trava o mantem armado nessa posicao inutil.
            #
            # A guarda ja existia no freekick.py (_chute_deve_estar_armado):
            #   "A BOLA PRECISA ESTAR NA FRENTE. Sem isto o criterio e simetrico
            #    e arma com a bola encostada nas costas do robo."
            # Terceira licao da bola parada que faltava trazer para o jogo
            # corrido - as outras duas foram a trava de armamento e nao exigir
            # alinhamento fino no instante do contato.
            _bxr = tt.ball.position_x - r.position_x
            _byr = tt.ball.position_y - r.position_y
            _frente_placa = (_bxr * cos(r.orientation)
                             + _byr * sin(r.orientation)) - 73.0
            _perto = d_bola < ALCANCE_CONTATO and _frente_placa > -20.0
            _ok_dir = para_frente or tipo_alvo == "passe"
            _travas = tt.estado.setdefault("chute_armado", {}) \
                if hasattr(tt, "estado") and tt.estado is not None else {}
            if (_travas.get(rid) and d_bola < ALCANCE_SOLTA_TRAVA
                    and _ok_dir and _frente_placa > -20.0):
                arma = True                      # segue armado
            else:
                arma = _perto and _ok_dir
            _travas[rid] = arma
            if os.environ.get("DIAG_JOGO"):
                # GEOMETRIA NO REFERENCIAL DO ROBO, que e o que o grSim arbitra:
                # ele dispara com xx < 31,5 mm (placa) e |yy| < 40 mm (largura).
                # Ver robot.cpp:120-128. Sem isto nao da para saber SE a bola
                # chega na placa - so que o robo esta "perto".
                _bx_r = tt.ball.position_x - r.position_x
                _by_r = tt.ball.position_y - r.position_y
                _co, _si = cos(r.orientation), sin(r.orientation)
                _xx = (_bx_r * _co + _by_r * _si) - 73.0
                _yy = -_bx_r * _si + _by_r * _co
                print("[JG] PLACA r%d papel=%s xx=%.0f yy=%.0f arma=%s dispara=%s"
                      % (rid, papel, _xx, _yy, arma,
                         (0.0 <= _xx < 31.5 and abs(_yy) < 40.0)), flush=True)
            na_face = mirado = arma          # mantidos so para o diagnostico
            if os.environ.get("DIAG_JOGO"):
                print("[JG] arma=%s face=%s mira=%s frente=%s d=%.0f tipo=%s"
                      % (arma, na_face, mirado, para_frente, d_bola, tipo_alvo),
                      flush=True)
        cmd.kick = forca if arma else 0.0
        # a bola so e obstaculo para quem NAO vai disputa-la
        cmd.ball = (papel != PAPEL_PORTADOR)
        cmd.field_border = True
        cmd.penalty_area = True
        comandos.append(cmd)
    return comandos



class Atack:
    def __init__(self, ally_robots, enemy_robots, ball, on_positive_half,
                 estado=None):
        self.name = "OurAtack"
        # Estado da jogada, injetado pela play. A tatica e reconstruida a cada
        # ciclo, entao guardar aqui seria perder - e o mesmo motivo pelo qual a
        # cobranca de falta usa EstadoFreekick. Hoje guarda so o alvo do passe.
        self.estado = estado if estado is not None else {}
        self.skills_factory = Skills("Movement")
        self.goal_center = CenterGoal()
        self.on_positive_half = on_positive_half
        self.ally_robots = ally_robots
        self.enemy_robots = enemy_robots
        self.ball = ball
        self.kick_threshold = 1500.0
        if self.on_positive_half:
            self.gk_angle = 3.14159
            self.gk_target = self.goal_center.GOAL_POSITIVE
            self.attack_goal = self.goal_center.GOAL_NEGATIVE
        else:
            self.gk_angle = 0.0
            self.gk_target = self.goal_center.GOAL_NEGATIVE
            self.attack_goal = self.goal_center.GOAL_POSITIVE

    def _cobertura(self, robot_id, ordem=0):
        """Quem nao e o eleito: cobre entre a bola e o NOSSO gol, ESPALHADO.

        Posicionamento OFENSIVO (o nao eleito adiante da bola, oferecendo linha
        de passe) foi tentado duas vezes e piorou nas duas com DOIS robos de
        linha - deixava o eleito sozinho e entrava no corredor do empurrao.

        O que faltava era espalhar. Com tres robos de linha, mandar os dois nao
        eleitos para o MESMO ponto medio trouxe o amontoado de volta: 3% -> 18%
        do tempo com dois deles a menos de 600 mm da bola. Agora cada um recebe
        um deslocamento lateral proprio, alternando o lado pela ordem.
        """
        nosso_gol = (self.goal_center.GOAL_POSITIVE if self.on_positive_half
                     else self.goal_center.GOAL_NEGATIVE)
        base_x = (self.ball.position_x + nosso_gol.x) / 2.0
        base_y = (self.ball.position_y + nosso_gol.y) / 2.0
        # perpendicular a linha bola -> nosso gol
        dxg = self.ball.position_x - nosso_gol.x
        dyg = self.ball.position_y - nosso_gol.y
        ng = hypot(dxg, dyg) or 1.0
        px, py = -dyg / ng, dxg / ng
        lado = 1.0 if (ordem % 2 == 0) else -1.0
        desloc = 800.0 * (1 + ordem // 2) * lado
        alvo_x = max(-4300.0, min(4300.0, base_x + px * desloc))
        alvo_y = max(-2800.0, min(2800.0, base_y + py * desloc))
        cmd = self.skills_factory.move_with_angle(
            robot_id=robot_id, target_x=alvo_x, target_y=alvo_y,
            vel_x=0.0, vel_y=0.0,
            angle=atan2(self.ball.position_y - alvo_y,
                        self.ball.position_x - alvo_x),
        )
        cmd.ball = True
        cmd.field_border = True
        cmd.penalty_area = True
        return cmd

    def _enemy_is_near_ball(self) -> list:
        robots_enemy_near_ball = []

        for robot_id_, robot_info in self.enemy_robots.items():
            dist_to_ball = hypot(
                robot_info.position_x - self.ball.position_x,
                robot_info.position_y - self.ball.position_y,
            )
            if dist_to_ball < 500.0:
                robots_enemy_near_ball.append(robot_id_)

        return robots_enemy_near_ball

    def _can_kick(self):
        if self.on_positive_half and self.ball.position_x < -self.kick_threshold:
            return True
        elif not self.on_positive_half and self.ball.position_x > self.kick_threshold:
            return True

        return False

    def _get_angle_to_goal(self, robot_id) -> float:
        robot = None
        for rid, robot_info in self.ally_robots.items():
            if rid == robot_id:
                robot = robot_info
                break

        goal_pos = self.attack_goal

        if robot is None:
            return 0.0

        dx = goal_pos.x - robot.position_x
        dy = goal_pos.y - robot.position_y

        angle = atan2(dy, dx)
        return angle

    def _get_angle_to_ball(self, robot_id) -> float:
        robot = None
        for rid, robot_info in self.ally_robots.items():
            if rid == robot_id:
                robot = robot_info
                break

        if robot is None:
            return 0.0

        dx = self.ball.position_x - robot.position_x
        dy = self.ball.position_y - robot.position_y

        angle = atan2(dy, dx)
        return angle

    def _go_to_goal(self, robot_id):
        """Dispatcher: decide se o robô deve posicionar atrás da bola (stage)
        ou avançar para empurrar a bola para o gol.

        - _go_to_ball: vai até atrás da bola e se alinha; trata a bola como
          obstáculo (robot_command.ball = True) enquanto posiciona.
        - _do_push: vai para a frente da bola e empurra; permite interação
          com a bola (robot_command.ball = False) e ativa o kicker quando
          aplicável.
        """
        bx, by = self.ball.position_x, self.ball.position_y
        gx, gy = self.attack_goal.x, self.attack_goal.y
        dx, dy = gx - bx, gy - by
        norm = hypot(dx, dy) or 1.0
        ux, uy = dx / norm, dy / norm

        behind_dist = 70.0
        stage_x = bx - ux * behind_dist
        stage_y = by - uy * behind_dist

        rx = ry = None
        for rid, robot_info in self.ally_robots.items():
            if rid == robot_id:
                rx, ry = robot_info.position_x, robot_info.position_y
                break

        # distância até stage/bola
        dist_to_stage = (
            hypot(rx - stage_x, ry - stage_y) if rx is not None else float("inf")
        )
        dist_to_ball = hypot(rx - bx, ry - by) if rx is not None else float("inf")

        # Se estiver perto o suficiente do stage ou da bola, faça o push,
        # caso contrário aproxime-se e alinhe-se atrás da bola.
        if dist_to_stage < 230.0 or dist_to_ball < 230.0:
            return self._do_push(robot_id, bx, by, ux, uy, dx, dy)
        else:
            return self._go_to_ball(robot_id, stage_x, stage_y, dx, dy)

    def _go_to_ball(self, robot_id, stage_x, stage_y, dx, dy):
        """
        Aproxima-se do ponto 'atrás da bola' e alinha-se na direção do gol.
        Enquanto posiciona, a bola é tratada como obstáculo (ball=False) para
        evitar comandos de empurrão prematuros.
        """
        angle = atan2(dy, dx)

        robot_command = self.skills_factory.move_with_angle(
            robot_id=robot_id,
            target_x=stage_x,
            target_y=stage_y,
            vel_x=0.0,
            vel_y=0.0,
            angle=angle,
        )

        robot_command.field_border = True
        robot_command.ball = True
        robot_command.ally_ids = [0, 1]
        robot_command.enemy_ids = self._enemy_is_near_ball()
        robot_command.penalty_area = True
        robot_command.deactivate_kick()

        return robot_command

    def _do_push(self, robot_id, bx, by, ux, uy, dx, dy):
        rx = ry = None
        for _rid, _ri in self.ally_robots.items():
            if _rid == robot_id:
                rx, ry = _ri.position_x, _ri.position_y
                break
        """
        Avança à frente da bola (na direção do gol) e empurra.
        Permite interação com a bola (ball=True) e ativa o kicker quando
        a condição de chute é satisfeita.
        """
        # BOLA EM VOO: para de avancar. Ver VEL_BOLA_CHUTADA.
        #
        # Continuando o empurrao depois do disparo, o robo alcanca a bola e a
        # freia (robot.cpp:168). Segurando a posicao, ela viaja.
        push_dist = 220.0
        if bola_ja_saiu(self.ball):
            target_x, target_y = rx, ry
        else:
            target_x = bx + ux * push_dist
            target_y = by + uy * push_dist

        angle = atan2(dy, dx)

        robot_command = self.skills_factory.move_with_angle(
            robot_id=robot_id,
            target_x=target_x,
            target_y=target_y,
            vel_x=0.0,
            vel_y=0.0,
            angle=angle,
        )

        robot_command.ball = False
        robot_command.field_border = True
        robot_command.ally_ids = [0, 1]
        robot_command.enemy_ids = self._enemy_is_near_ball()
        robot_command.penalty_area = True

        if self.on_positive_half:
            for rid, robot_info in self.ally_robots.items():
                if self.ball.position_x > robot_info.position_x and rid == robot_id:
                    robot_command.ball = True
        else:
            for rid, robot_info in self.ally_robots.items():
                if self.ball.position_x < robot_info.position_x and rid == robot_id:
                    robot_command.ball = True

        # CHUTE: forca medida e ARMADO CEDO, igual ao da Defense.
        #
        # Era activate_kick() (1,5 m/s) e so com a bola alem de x=+-1500. Duas
        # coisas erradas:
        #  - 1,5 m/s nao tira a bola de perto. Atrito medido no grSim, na fase
        #    da bola parada: 3,0 m/s percorre 1633 mm, 6,0 percorre 4220. A
        #    curva e abrupta, e abaixo de 6 a bola morre no caminho.
        #  - o limiar de 1500 quase nunca era satisfeito em jogo, entao o chute
        #    praticamente nao existia.
        #
        # E a licao do §18 da bola parada: o grSim so dispara no instante do
        # contato, e essa janela dura UMA amostra. Armar cedo nao custa nada -
        # o simulador ignora ate haver contato. Recusar armar custa a jogada.
        _db = (hypot(rx - self.ball.position_x, ry - self.ball.position_y)
               if rx is not None else 9999.0)
        robot_command.kick = FORCA_CHUTE if _db < FORCA_CHUTE_ALCANCE else 0.0

        return robot_command

    def _robot_is_stable(self, robot_id) -> bool:
        for robot_id_, robot_info in self.ally_robots.items():
            if robot_id_ == robot_id:
                if abs(robot_info.velocity_x) < 25 and abs(robot_info.velocity_y) < 25:
                    return True
                break

        return False



    def _comandos_por_papel(self):
        # UMA implementacao so, no nivel do modulo. As duas copias que
        # existiam aqui ja tinham divergido: a da Defense nunca chamou
        # 'alvo_do_chute', entao defendendo o time nao escolhia alvo e
        # nunca passava 'alvo_chute' adiante - o portador mirava sempre
        # no gol. Consertar uma corrigia metade do jogo.
        return montar_comandos(self)

    def execute(self):
        """Comandos do ciclo. A logica esta em _comandos_por_papel."""
        comandos = []
        if 0 in self.ally_robots:
            gk = Goalkeeper(self.ally_robots[0], self.ball,
                            self.on_positive_half,
                            self.ally_robots, self.enemy_robots)
            comandos.append(gk.execute(self.gk_target, self.ball))
        comandos.extend(self._comandos_por_papel())
        return comandos

class Defense:
    def __init__(self, ally_robots, ball, on_positive_half, enemy_robots=None,
                 estado=None):
        self.name = "OurDefense"
        self.estado = estado if estado is not None else {}
        self.skills_factory = Skills("Movement")
        self.goal_center = CenterGoal()
        self.on_positive_half = on_positive_half
        self.ally_robots = ally_robots
        self.enemy_robots = enemy_robots or {}
        self.ball = ball

        if self.on_positive_half:
            self.angle = 3.14159
            self.gk_target = self.goal_center.GOAL_POSITIVE
        else:
            self.angle = 0.0
            self.gk_target = self.goal_center.GOAL_NEGATIVE

    # _go_to_ball foi REMOVIDO: era 'pass', metodo morto que nunca foi chamado.
    #
    # E o mesmo defeito P12 que a cobranca de falta ja teve. Metodo morto com
    # nome sugestivo e pior que nenhum metodo: quem le a classe conclui que
    # existe logica de ir a bola, e nao existe. Quem for a bola hoje e o robo
    # eleito por eleger_atacante, no execute() logo abaixo.

    def _comandos_por_papel(self):
        # UMA implementacao so, no nivel do modulo. As duas copias que
        # existiam aqui ja tinham divergido: a da Defense nunca chamou
        # 'alvo_do_chute', entao defendendo o time nao escolhia alvo e
        # nunca passava 'alvo_chute' adiante - o portador mirava sempre
        # no gol. Consertar uma corrigia metade do jogo.
        return montar_comandos(self)

    def execute(self):
        """Comandos do ciclo. A logica esta em _comandos_por_papel."""
        comandos = []
        if 0 in self.ally_robots:
            gk = Goalkeeper(self.ally_robots[0], self.ball,
                            self.on_positive_half,
                            self.ally_robots, self.enemy_robots)
            comandos.append(gk.execute(self.gk_target, self.ball))
        comandos.extend(self._comandos_por_papel())
        return comandos
