from utils.math_util import Vector2D
import os

from math import atan2, hypot, pi

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

    # PORTADOR: o mais adiantado, com a mesma fixacao de antes - trocar de dono
    # no meio da corrida desperdicia a corrida (medido: 8 trocas em 247 ciclos,
    # em blocos de 3 a 13 ciclos, e ninguem chegava a empurrar a bola).
    portador = max(linha, key=_adiantado)
    if estado is not None:
        ant = estado.get("portador")
        if ant is not None and ant in ally_robots and ant != portador:
            if _adiantado(portador) - _adiantado(ant) < VANTAGEM_TROCA:
                portador = ant
        estado["portador"] = portador

    restantes = [rid for rid in linha if rid != portador]
    papeis = {portador: PAPEL_PORTADOR}
    if not restantes:
        return papeis

    # BUSCADOR: entre os de tras, o mais proximo da bola. Tambem fixado.
    buscador = min(restantes, key=lambda rid: _dist_bola(ally_robots[rid], ball))
    if estado is not None:
        ant = estado.get("buscador")
        if ant is not None and ant in restantes and ant != buscador:
            if (_dist_bola(ally_robots[ant], ball)
                    - _dist_bola(ally_robots[buscador], ball)) < VANTAGEM_TROCA:
                buscador = ant
        estado["buscador"] = buscador

    papeis[buscador] = PAPEL_APOIO          # APOIO = o buscador
    for rid in restantes:
        if rid != buscador:
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
VANTAGEM_TROCA = 600.0
RAIO_POSSE_PORTADOR = 200.0
# Quanto ele avanca ALEM da bola ao empurrar/tocar - atravessa em vez de parar.
AVANCO_PORTADOR = 500.0
# Quanto a frente da bola o portador se oferece quando nao e ele que busca.
PORTADOR_ESPACO = 1500.0
# Onde a cobertura fica na reta bola->nosso gol: 0 e na bola, 1 e no gol.
# 0,45 a deixa mais perto da bola que do gol, dentro do corredor do chute e com
# tempo de reagir ao rebote.
COBERTURA_FRACAO = 0.45
RAIO_MIRA = 400.0
# Portao fisico: a face do chutador tem 80 mm a 73 mm do centro (~29 graus).
# 25 graus de face deixa margem para o rastreio; 20 para a mira.
TOL_FACE = 0.44
TOL_MIRA = 0.35
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
        if bola_ja_saiu(ball):
            fx, fy = onde_a_bola_vai(ball, 0.5)
            dxp, dyp = fx - rx, fy - ry
            n_p = hypot(dxp, dyp) or 1.0
            return _no_campo(fx + (dxp / n_p) * AVANCO_PORTADOR,
                             fy + (dyp / n_p) * AVANCO_PORTADOR) + (True,)
        return _no_campo(bx + ux * AVANCO_PORTADOR,
                         by + uy * AVANCO_PORTADOR) + (True,)

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

    # O PORTADOR NAO VAI MAIS BUSCAR. Ele ataca o espaco e espera a bola.
    #
    # Quem busca e o apoio (buscador); em DELES e DISPUTA a cobertura vai junto.
    # Se o portador tambem descesse, ninguem estaria a frente quando a bola
    # fosse recuperada, e era isso que acontecia: zero quadros alem de x=3500 em
    # todas as execucoes, em todos os lotes desta fase.
    #
    # Com a bola longe e nao nossa, ele se oferece: adiante da bola, no meio.
    if situacao in (SITUACAO_DELES, SITUACAO_SOLTA) and d_bola > RAIO_POSSE_PORTADOR:
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
        # SO assume a linha de tiro COM a bola. Fora isso olha para ela.
        #
        # O gatilho era a distancia (400 mm), e a linha de tiro sai da posicao
        # da BOLA: com o robo do lado errado ela aponta para longe dela. Medido
        # nos replays: o portador com 31-33 graus de erro medio e de costas em
        # 20% do tempo, contra 2-3 graus do apoio e da cobertura.
        if papel == PAPEL_PORTADOR and _d0 < RAIO_POSSE_PORTADOR:
            # JA ESTA ATRAS E ALINHADO: o corpo assume a linha de tiro.
            mira = alvo_chute or (gol_ataque.x, gol_ataque.y)
            ang = atan2(mira[1] - tt.ball.position_y,
                        mira[0] - tt.ball.position_x)
        elif papel == PAPEL_PORTADOR:
            # AINDA CHEGANDO: olha para a BOLA, a partir de onde ele esta.
            #
            # Antes o portador recebia a linha bola->gol mesmo longe da bola.
            # Esse angulo sai da posicao da BOLA, nao da dele: enquanto se
            # aproxima ele gira para uma pose sem relacao com o proprio
            # deslocamento - de fora parece perdido - e chega de lado na bola.
            # Contato lateral manda a bola para qualquer lugar. E com a linha
            # bloqueada 'mira' virava o gol: era dai o "so chuta para frente".
            #
            # Olhando para a bola, a face do chutador chega de frente nela, e a
            # linha de tiro so entra quando ela realmente vale.
            r_ = tt.ally_robots[rid]
            ang = atan2(tt.ball.position_y - r_.position_y,
                        tt.ball.position_x - r_.position_x)
        else:
            ang = atan2(tt.ball.position_y - alvo_y,
                        tt.ball.position_x - alvo_x)

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
        forca = 2.5 if tipo_alvo == "passe" else FORCA_CHUTE
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
        if alvo_chute is None or d_bola >= FORCA_CHUTE_ALCANCE:
            arma = False
        else:
            ang_alvo = atan2(alvo_chute[1] - tt.ball.position_y,
                             alvo_chute[0] - tt.ball.position_x)
            ang_bola = atan2(tt.ball.position_y - r.position_y,
                             tt.ball.position_x - r.position_x)
            na_face = abs(_norm_ang(ang_bola - r.orientation)) < TOL_FACE
            mirado = abs(_norm_ang(ang_alvo - r.orientation)) < TOL_MIRA
            arma = na_face and mirado
            if os.environ.get("DIAG_JOGO") and papel == PAPEL_PORTADOR:
                print("[JG] arma=%s face=%s mira=%s d=%.0f tipo=%s"
                      % (arma, na_face, mirado, d_bola, tipo_alvo), flush=True)
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
                            self.on_positive_half)
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
                            self.on_positive_half)
            comandos.append(gk.execute(self.gk_target, self.ball))
        comandos.extend(self._comandos_por_papel())
        return comandos
