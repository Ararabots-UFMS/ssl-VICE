from utils.math_util import Vector2D
import json
import os

from math import atan2, hypot

from strategy.skills.skills import Skills
from strategy.skills import aproximacao, chute, geometria, posicionamento
from strategy.skills import bola as skill_bola
from strategy.tatics.goalkeeper import Goalkeeper


# CONSTANTES E PREDICADOS DE CHUTE E DE BOLA: moram na CAMADA DE SKILLS.
#
# O racional medido de cada numero (por que 6,0 m/s, por que armar a 300 mm, por
# que 250 mm/s e nao 800) esta em skills/chute.py e skills/bola.py, junto da
# funcao que usa o numero. Aqui ficam so os nomes, para nao quebrar quem importa
# deste modulo.
FORCA_CHUTE = chute.FORCA_CHUTE
FORCA_CHUTE_ALCANCE = chute.FORCA_CHUTE_ALCANCE
VEL_BOLA_CHUTADA = skill_bola.VEL_BOLA_CHUTADA
bola_ja_saiu = skill_bola.bola_ja_saiu


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

# OS TRES PAPEIS, e o que cada nome QUER DIZER.
#
# Os nomes estavam trocados: o rotulo 'apoio' era dado a quem ia a BOLA e o
# rotulo 'portador' a quem ficava adiantado esperando. Nao era so cosmetico -
# 'alvo_do_chute' escolhe o receptor do passe entre os PAPEL_APOIO, ou seja
# escolhia justamente quem estava indo disputar a bola. Ver distribuir_papeis.
#
#   PORTADOR    quem tem a bola ou vai busca-la. UM por ciclo.
#   APOIO       quem ajuda sem a bola: ataque (oferece-se) ou marcacao.
#   COBERTURA   entre a bola e o nosso gol.
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


# Previsao da bola: skills/bola.py (atencao a fonte - o Kalman subnotifica).
onde_a_bola_vai = skill_bola.onde_a_bola_vai


def _dist_bola(robo, ball):
    return hypot(robo.position_x - ball.position_x,
                 robo.position_y - ball.position_y)


PAPEIS_ARQUIVO = "/tmp/ararabots_papeis.json"


def _gravar_papeis(papeis):
    """Publica o papel de cada robo para o gravador do replay (ararabots.py)."""
    try:
        with open(PAPEIS_ARQUIVO, "w") as f:
            json.dump({str(k): v for k, v in papeis.items()}, f)
    except OSError:
        pass


def distribuir_papeis(ally_robots, ball, situacao, estado=None, sentido=1.0):
    """Quem vai a bola, quem apoia, quem cobre. SEMPRE os tres papeis.

    OS NOMES ESTAVAM TROCADOS, E ISSO NAO ERA COSMETICO
    ---------------------------------------------------
    Ate aqui o rotulo PAPEL_APOIO era dado a quem ia a BOLA (chamado
    'buscador' nos comentarios) e o PAPEL_PORTADOR a quem ficava adiantado
    esperando. Um consumidor leu o rotulo pelo que ele DIZ: 'alvo_do_chute'
    escolhe o receptor do passe entre os PAPEL_APOIO - isto e, escolhia
    justamente quem estava indo disputar a bola.

    MEDIDO offline, sonda de decisao sem simulador (as duas funcoes sao puras):
        receptor do passe == robo mais proximo da bola     3 de 3 cenarios
    Com o robo a 100 mm da bola, o 'passe' virava um alvo em cima do proprio
    chutador. E como TODA direcao do ciclo sai de 'alvo_chute' - a orientacao
    do corpo, o ponto de atravessar, o 'espaco a frente' do apoio -, a
    consequencia era o time apontar e andar PARA TRAS:

        cenario 'posse nossa'  corpo do apoio   -139,9 graus (nosso gol)
        cenario 'largada'      alvo do apoio    (-1500, 0) com a bola em (0,0)

    E a trava de seguranca entao recusava armar o chute, porque a direcao de
    saida apontava para a nossa meta. Laco fechado: gol bloqueado -> passe
    degenerado -> todas as direcoes invertidas -> nao chuta. Isto explica o
    'alvo_chute=passe em 250 de 250 ciclos' melhor do que a folga de linha,
    porque o segmento bola -> robo-mais-proximo e curto e quase sempre livre.

    O QUE CADA PAPEL E AGORA
    ------------------------
      PORTADOR   tem a bola ou vai busca-la. Escolhido PRIMEIRO, o mais
                 proximo dela. (Era o 'buscador'.)
      APOIO      ajuda sem a bola - ataque ou marcacao, conforme o cenario.
                 O mais adiantado entre os que sobram. (Era o 'portador'.)
      COBERTURA  entre a bola e o nosso gol.

    O PORTADOR E ESCOLHIDO PRIMEIRO, E E O MAIS PROXIMO DA BOLA
    -----------------------------------------------------------
    Isto nao mudou, e e a inversao que consertou a largada. Antes o mais
    ADIANTADO era escolhido primeiro e quem ia a bola saia dos que sobravam:
    com a bola no centro, o nosso robo 1 a 1200 mm dela e os robos 2 e 3 a
    ~2500 mm, o robo 1 ficava parado ocupando espaco e mandavamos disputar
    quem estava MAIS LONGE. Medido: em 2 de 3 replays o amarelo tocava a bola
    aos 0,7 s e chutava 5400 mm direto para o nosso fundo, com UM unico toque
    na partida inteira.

    A mesma regra produz os dois comportamentos certos, pela geometria:
      - bola no centro na largada -> o mais proximo e o da frente, e ele vai;
      - bola sobrando atras com o atacante la em cima -> o mais proximo e um
        dos de tras, e o adiantado fica livre para o espaco.

    QUANTOS VAO A BOLA, por situacao:
      SOLTA          UM  - o portador. Dois atras da mesma bola solta e
                     desperdicio: o outro fica livre para o espaco.
      DELES/DISPUTA  o portador disputa; a cobertura protege o gol.
      NOSSA          UM - o portador NAO larga a bola (ver alvo_do_papel); o
                     apoio sobe para receber.

    OS TRES SEMPRE EXISTEM
    ----------------------
    Pedido do Felipe: "faca apoio, portador e cobertura sempre". Com tres
    robos de linha (o cenario 'jogo' tem quatro: goleiro + tres) sai exatamente
    um de cada. Com MENOS robos que papeis a prioridade e

        PORTADOR -> COBERTURA -> APOIO

    porque alguem tem de ir a bola, e deixar a linha do gol descoberta custa
    mais que perder a linha de passe. Antes, com dois de linha, saiam portador
    e apoio e NINGUEM cobria.

    'sentido' e +1 quando atacamos +x e -1 quando atacamos -x: e o que define
    "adiantado".
    """
    linha = sorted(rid for rid in ally_robots if rid != 0)
    if not linha:
        return {}

    # EXPERIMENTO A2-B: PAPEIS FIXOS POR ID (ARARABOTS_PAPEIS_FIXOS=1).
    #
    # Isola a ELEICAO e a HISTERESE de uma vez: o robo de menor id e sempre o
    # portador, o seguinte sempre o apoio, o resto cobertura. Nada troca,
    # nunca. T-2 ja respondeu (engajamento igual, amontoamento de 1% para 42%),
    # entao a bandeira pode sair - fica porque o lote que a respondeu esta
    # registrado no ESTADO_ATUAL.md e o custo de manter e uma linha.
    #
    # NAO E PARA FICAR: papeis fixos ignoram a geometria, entao na bola
    # sobrando atras quem busca pode ser o mais distante - que e exatamente o
    # defeito da largada que a inversao portador-primeiro corrigiu.
    # A bandeira e um ARQUIVO, nao uma variavel de ambiente: variavel lida pela
    # ESTRATEGIA precisa ser exportada no 'ros_d' do ararabots.sh, que fica fora
    # de src/strategy/.
    # O menu de zagueiro exporta a variavel via ros_d para testar a cobertura
    # sozinha. O arquivo continua disponivel para experimentos manuais.
    if (os.environ.get("ARARABOTS_FORCAR_COBERTURA") == "1"
            or os.path.exists("/tmp/ararabots_forcar_cobertura")):
        return {rid: PAPEL_COBERTURA for rid in linha}

    if (os.environ.get("ARARABOTS_PAPEIS_FIXOS")
            or os.path.exists("/tmp/ararabots_papeis_fixos")):
        papeis_fixos = {}
        for i, rid in enumerate(linha):
            if i == 0:
                papeis_fixos[rid] = PAPEL_PORTADOR
            elif i == 1:
                papeis_fixos[rid] = PAPEL_APOIO
            else:
                papeis_fixos[rid] = PAPEL_COBERTURA
        return papeis_fixos

    def _adiantado(rid):
        return ally_robots[rid].position_x * sentido

    # ---- PORTADOR: o mais proximo da bola, com histerese
    portador = min(linha, key=lambda rid: _dist_bola(ally_robots[rid], ball))
    if estado is not None:
        ant = estado.get("portador")
        if ant is not None and ant in linha and ant != portador:
            travado = estado.get("portador_ciclos", 0) < CICLOS_MIN_PAPEL
            if travado or (_dist_bola(ally_robots[ant], ball)
                           - _dist_bola(ally_robots[portador], ball)
                           < VANTAGEM_TROCA):
                portador = ant
        # T-3: QUAL TROCA CUSTA CARO. So log, nenhum comportamento muda.
        #
        # "a troca de papel custa" e grosso demais para virar correcao: trocar o
        # apoio com a bola longe e barato; trocar o PORTADOR quando ele ja esta
        # a meio caminho da bola joga fora a corrida inteira. O log registra a
        # distancia do robo que SAI e do que ENTRA, no instante da troca.
        if ant is not None and ant != portador and os.environ.get("DIAG_JOGO"):
            print("[TR] troca=portador sai=r%s d_sai=%.0f entra=r%s d_entra=%.0f"
                  % (ant, _dist_bola(ally_robots[ant], ball) if ant in ally_robots else -1,
                     portador, _dist_bola(ally_robots[portador], ball)), flush=True)
        estado["portador_ciclos"] = (
            estado.get("portador_ciclos", 0) + 1 if portador == ant else 0)
        estado["portador"] = portador

    sobram = [rid for rid in linha if rid != portador]
    if not sobram:
        # UM robo de linha so: ele e o portador. Nao ha o que distribuir.
        return {portador: PAPEL_PORTADOR}

    # ---- COBERTURA antes do APOIO quando falta gente (ver docstring)
    if len(sobram) == 1:
        return {portador: PAPEL_PORTADOR, sobram[0]: PAPEL_COBERTURA}

    # ---- APOIO: o mais adiantado entre os que sobram, com histerese
    apoio = max(sobram, key=_adiantado)
    if estado is not None:
        ant = estado.get("apoio")
        if ant is not None and ant in sobram and ant != apoio:
            travado = estado.get("apoio_ciclos", 0) < CICLOS_MIN_PAPEL
            if travado or _adiantado(apoio) - _adiantado(ant) < VANTAGEM_TROCA:
                apoio = ant
        if ant is not None and ant != apoio and os.environ.get("DIAG_JOGO"):
            print("[TR] troca=apoio sai=r%s d_sai=%.0f entra=r%s d_entra=%.0f"
                  % (ant, _dist_bola(ally_robots[ant], ball) if ant in ally_robots else -1,
                     apoio, _dist_bola(ally_robots[apoio], ball)), flush=True)
        estado["apoio_ciclos"] = (
            estado.get("apoio_ciclos", 0) + 1 if apoio == ant else 0)
        estado["apoio"] = apoio

    papeis = {portador: PAPEL_PORTADOR, apoio: PAPEL_APOIO}
    for rid in linha:
        if rid not in papeis:
            papeis[rid] = PAPEL_COBERTURA
    return papeis



_livre_do_lado = geometria.livre_do_lado


FOLGA_LINHA = geometria.FOLGA_LINHA

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
# Nosso terco defensivo: campo de 9000, gol em -4500, entao 3000 mm de raio
# cobre o terco. Dentro disto a prioridade e AFASTAR a bola.
TERCO_DEFENSIVO = 3000.0
RAIO_MIRA = 400.0
TOL_MIRA = 0.35
# Dentro disto o robo ja aponta na direcao do chute em vez de olhar para a bola.
RAIO_ORIENTA_CHUTE = 700.0
PRESSAO_RAIO = 400.0
PRESSAO_MIN = 1
LIMITE_Y = 2600.0


# Predicado de linha: skills/geometria.py. ATENCAO ao contrato - os quatro
# primeiros argumentos sao COORDENADAS, nao um ponto e um versor (o goleiro
# chamava com versor e nao conferia linha nenhuma).

# Constantes que passaram para a CAMADA DE SKILLS, junto da funcao que as
# usa. O racional medido de cada uma esta la; aqui fica so o nome, porque
# constante medida escrita em dois lugares e defeito esperando acontecer.
AVANCO_PORTADOR = aproximacao.AVANCO_PORTADOR
AVANCO_SOLTA = aproximacao.AVANCO_SOLTA
RECUO_CONTORNO = aproximacao.RECUO_CONTORNO
PONTO_CHUTE = aproximacao.PONTO_CHUTE
VIES_LATERAL = aproximacao.VIES_LATERAL
RAIO_ENCAIXE = aproximacao.RAIO_ENCAIXE
ATRAVESSA_CHUTE = aproximacao.ATRAVESSA_CHUTE
LATERAL_CONTORNO = aproximacao.LATERAL_CONTORNO
PORTADOR_ESPACO = posicionamento.PORTADOR_ESPACO
BLOQUEIO_DIST = posicionamento.BLOQUEIO_DIST
MARCACAO_DIST = posicionamento.MARCACAO_DIST
COBERTURA_FRACAO = posicionamento.COBERTURA_FRACAO
TOL_PARA_TRAS = chute.TOL_PARA_TRAS
ALCANCE_CONTATO = chute.ALCANCE_CONTATO
ALCANCE_SOLTA_TRAVA = chute.ALCANCE_SOLTA_TRAVA
linha_livre = geometria.linha_livre


def alvo_do_chute(ball, gol_ataque, ally_robots, enemy_robots, papeis, estado=None):
    """Para onde o portador deve mandar a bola: o gol, o APOIO, ou lugar nenhum.

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
    # O RECEPTOR TEM DE ESTAR A FRENTE DA BOLA.
    #
    # DEFEITO QUE ISTO CORRIGE, e era o pior achado da auditoria de papeis.
    # O receptor era escolhido entre os PAPEL_APOIO, e o PAPEL_APOIO era o robo
    # que ia a BOLA (os nomes estavam trocados - ver distribuir_papeis). Ou
    # seja: o alvo do passe era a posicao de quem ia chutar.
    #
    # MEDIDO offline (sonda de decisao, sem simulador):
    #     receptor == robo mais proximo da bola      3 de 3 cenarios
    #     'posse nossa': receptor a 100 mm da bola
    # Como o segmento bola -> robo-mais-proximo e curto, 'linha_livre' quase
    # sempre aprovava: 'passe' ganhava sempre que o gol estivesse fechado. E
    # todas as direcoes do ciclo saem de 'alvo_chute', entao o corpo apontava
    # para tras (-139,9 graus medidos) e a trava de seguranca recusava armar.
    #
    # Com os papeis nomeados corretamente o receptor passa a ser o APOIO - o
    # mais adiantado, que e quem se oferece. A guarda de AVANCO vem da cobranca
    # de falta, que ja tinha a licao paga em _companheiro_para_passe: "menos que
    # isso e passe lateral dentro da propria area". Sem ela, um apoio que
    # recuou vira alvo e o passe sai para tras.

    if estado is not None:
        congelado = estado.get("alvo_passe")
        if congelado is not None:
            rid_c = congelado[2]
            atual = ally_robots.get(rid_c)
            if (atual is not None and papeis.get(rid_c) == PAPEL_APOIO
                    and posicionamento.pode_receber(atual, bx, by, gol_ataque)
                    and linha_livre(bx, by, atual.position_x, atual.position_y,
                                    enemy_robots)):
                return congelado[0], congelado[1], "passe"
            estado.pop("alvo_passe", None)

    for rid, papel in papeis.items():
        if papel != PAPEL_APOIO or rid not in ally_robots:
            continue
        a = ally_robots[rid]
        if (posicionamento.pode_receber(a, bx, by, gol_ataque)
                and linha_livre(bx, by, a.position_x, a.position_y, enemy_robots)):
            if estado is not None:
                estado["alvo_passe"] = (a.position_x, a.position_y, rid)
            return a.position_x, a.position_y, "passe"
    return None, None, "bloqueado"





# Constantes que passaram para a CAMADA DE SKILLS, junto da funcao que as
# usa. O racional medido de cada uma esta la; aqui fica so o nome, porque
# constante medida escrita em dois lugares e defeito esperando acontecer.
AVANCO_MINIMO_PASSE = posicionamento.AVANCO_MINIMO_PASSE
APOIO_RECUO = posicionamento.APOIO_RECUO
APOIO_AVANCO = posicionamento.APOIO_AVANCO
APOIO_ABERTURA = posicionamento.APOIO_ABERTURA
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

    _no_campo = geometria.no_campo

    # ---------------------------------------------------------------- APOIO
    # ----------------------------------------------------------- PORTADOR
    #
    # QUEM VAI A BOLA E QUEM A TEM. Era o rotulo PAPEL_APOIO ('buscador').
    #
    # Todo o maquinario abaixo - interceptacao, contorno continuo, ponto de
    # chute, desvio lateral que zera na chegada - veio deste ramo e NAO foi
    # alterado: cada constante dele tem medicao no comentario. O que mudou e
    # o nome do papel e uma coisa de comportamento, logo abaixo (NOSSA).
    if papel == PAPEL_PORTADOR:
        if situacao == SITUACAO_SOLTA:
            # vai para onde a bola VAI PARAR, nao para onde ela esta: chegar
            # depois dela nao adianta.
            fx, fy = onde_a_bola_vai(ball, 1.0)
            return _no_campo(fx, fy) + (False,)

        # COM A BOLA NOSSA, O PORTADOR NAO LARGA A BOLA.
        #
        # MUDANCA DE COMPORTAMENTO, e e a razao de ser desta correcao. Neste
        # ramo havia o oposto: com a situacao NOSSA quem estava COM a bola era
        # mandado 1800 mm para a frente ('vira segundo atacante') enquanto o
        # robo adiantado era mandado NA bola. Os dois trocavam de intencao e
        # ninguem ficava com ela.
        #
        # MEDIDO offline (sonda de decisao, cenario 'posse nossa'):
        #     r1 a 100 mm da bola   -> alvo (2300, 800)   = 1,8 m para a frente
        #     r2 a 2484 mm da bola  -> alvo (1000, -800)  = de volta na bola
        # E e exatamente o que o replay mostrava, e esta escrito no proprio
        # arquivo: "o portador recebe a bola, toca nela e nao faz nada, ou se
        # afasta da bola".
        #
        # Agora NOSSA nao tem ramo proprio: cai no tratamento normal de bola
        # (atravessar na direcao do alvo), que e o que se faz com a posse.
        # Quem sobe para receber e o APOIO.
        # BOLA JA CHUTADA: INTERCEPTA, NAO PERSEGUE.
        #
        # Este tratamento existia, mas ESCRITO DEPOIS DO 'return' do contorno,
        # logo abaixo - codigo inalcancavel. O portador tinha a guarda
        # (ver o ramo PORTADOR); o buscador nao.
        #
        # POR QUE IMPORTA: robot.cpp:168 SUBTRAI velocidade da bola no contato.
        # Quem alcanca a bola em voo a FREIA. E o defeito que fez nascer o
        # VEL_BOLA_CHUTADA - medimos a bola partindo a 5929 mm/s e andando
        # 200 mm porque o proprio robo a alcancou e segurou.
        #
        # Com a bola viajando nao ha o que contornar: o lado por onde se chega
        # deixa de ser escolha nossa e passa a ser a trajetoria dela. Mirar o
        # ponto de interceptacao, e alem dele para nao chegar freando.
        if bola_ja_saiu(ball):
            _avanco_int = (AVANCO_SOLTA if situacao == SITUACAO_SOLTA
                           else AVANCO_PORTADOR)
            return aproximacao.ponto_de_interceptacao(
                rx, ry, ball, _avanco_int) + (True,)

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
        avanco_base = AVANCO_SOLTA if situacao == SITUACAO_SOLTA else AVANCO_PORTADOR
        # O contorno continuo, o ponto de chute e o desvio lateral que zera na
        # chegada vivem em skills/aproximacao.py, com as medicoes que os geraram.
        return aproximacao.ponto_de_aproximacao(
            rx, ry, bx, by, _dir_alvo, avanco_base) + (True,)


    if papel == PAPEL_COBERTURA:
        return posicionamento.cobertura_defensiva(
            bx, by, nosso_gol, ordem) + (False,)

    # -------------------------------------------------------------- APOIO
    #
    # QUEM AJUDA SEM A BOLA. Era o rotulo PAPEL_PORTADOR.
    #
    # Pedido do Felipe: "o papel apoio so ajuda ou na marcacao ou no ataque".
    # Entao o ramo decide por CENARIO, e nao por rotulo:
    #   ataque    (bola nossa, ou bola no campo deles) -> oferece-se adiantado
    #                                                    e aberto, para receber
    #   marcacao  (bola deles/disputa, ou bola no nosso campo) -> tira o angulo
    #   solta     -> ocupa o espaco a frente
    #
    # A DIRECAO AQUI SAI DO GOL DE ATAQUE, NAO DE 'alvo_chute'. Se saisse do
    # alvo do chute, e o alvo do chute e o proprio apoio (ele e o receptor do
    # passe), o apoio se posicionaria em relacao a si mesmo - a mesma
    # auto-referencia que produziu o passe degenerado. Ver alvo_do_chute.

    # versor do ATAQUE puro (bola -> gol deles) e a sua perpendicular
    _dgx, _dgy = gol_ataque.x - bx, gol_ataque.y - by
    _ng = hypot(_dgx, _dgy) or 1.0
    _uax, _uay = _dgx / _ng, _dgy / _ng
    _pax, _pay = -_uay, _uax

    # SE A BOLA E MINHA, EU DISPUTO - nao importa o rotulo deste ciclo.
    #
    # Guarda preservada do ramo antigo: a situacao e geometrica e oscila com o
    # rastreio (raio de posse 250 mm), e a histerese de papel dura 20 ciclos.
    # Nesse intervalo o apoio pode ser quem esta em cima da bola; mandar ele
    # se posicionar seria abandonar a bola que ele acabou de ganhar.
    _meu = all(_dist_bola(r, ball) <= hypot(o.position_x - bx, o.position_y - by) + 1.0
               for rid_o, o in ally_robots.items()
               if rid_o != 0 and rid_o != rid)
    if _dist_bola(r, ball) < ENGAJA_RAIO and _meu:
        if alvo_chute is not None:
            _avx, _avy = alvo_chute[0] - bx, alvo_chute[1] - by
            _nav = hypot(_avx, _avy) or 1.0
            _dx, _dy = _avx / _nav, _avy / _nav
        else:
            _dx, _dy = _uax, _uay
        return _no_campo(bx + _dx * AVANCO_PORTADOR,
                         by + _dy * AVANCO_PORTADOR) + (True,)

    # ---- MARCACAO: a bola e deles, ou esta na zona de perigo
    #
    # A DISPUTA NAO ENTRA AQUI, de proposito. Ela e 51% do tempo (medido em
    # 8802 quadros); se disputa virasse motivo para o apoio recuar, ele passaria
    # metade do jogo marcando e ninguem estaria a frente quando a bola fosse
    # recuperada - que e o defeito medido desta fase (ZERO quadros com robo
    # nosso alem de x=3500, em todas as execucoes de todos os lotes).
    #
    # Zona de perigo e o mesmo TERCO_DEFENSIVO que liga a saida de bola: a
    # 3000 mm do nosso gol a prioridade e tirar o perigo, nao construir.
    _zona_de_perigo = hypot(bx - nosso_gol.x, by - nosso_gol.y) < TERCO_DEFENSIVO
    if situacao == SITUACAO_DELES or _zona_de_perigo:
        # PERDEU A DISPUTA? ENTAO TAPA A LINHA DE CHUTE.
        #
        # Bloco preservado do ramo antigo, com a medicao que o motivou:
        # a bola saindo do pe deles direto para o nosso fundo, 5461 e 5377 mm
        # em linha reta, com UM unico toque na partida inteira. Com o portador
        # disputando e a cobertura fechando o gol, o terceiro corpo tira o
        # angulo de passe.
        if inimigos:
            dono = min(inimigos.values(),
                       key=lambda e: hypot(e.position_x - bx, e.position_y - by))
            if hypot(dono.position_x - bx, dono.position_y - by) < RAIO_POSSE:
                return posicionamento.bloquear_linha(bx, by, nosso_gol) + (False,)
            # Ninguem com a posse: marca a AMEACA, que e o adversario mais
            # adiantado no nosso campo - quem recebe o passe seguinte. Fica
            # entre ele e o nosso gol, encostado, nao em cima dele: sem
            # dribbler o contato e o que decide a direcao, e queremos o
            # angulo, nao a dividida.
            ameaca = posicionamento.ameaca_mais_perigosa(inimigos, nosso_gol)
            if ameaca is not None:
                return posicionamento.marcar(ameaca, nosso_gol) + (False,)
        # sem adversario em campo (cenarios de teste): cobre a linha
        return posicionamento.bloquear_linha(bx, by, nosso_gol) + (False,)

    # ---- SOLTA: ocupa o espaco a frente
    #
    # Com a bola sobrando, alguem tem de estar a frente quando ela for
    # recuperada: medimos ZERO quadros com robo nosso alem de x=3500 em todas
    # as execucoes desta fase, porque todo mundo descia.
    #
    # O 'a frente' agora e a direcao do ATAQUE. Antes saia de 'alvo_chute', e
    # com o passe degenerado isso apontava para tras: MEDIDO offline, cenario
    # 'largada', o alvo do apoio era (-1500, 0) com a bola em (0,0) - ele
    # 'ocupava o espaco' recuando 1,5 m na direcao do nosso gol.
    if situacao == SITUACAO_SOLTA:
        return posicionamento.ocupar_espaco(bx, by, _uax, _uay) + (False,)

    # ---- ATAQUE (bola nossa): oferece-se adiantado e ABERTO
    #
    # 1800/1600, e nao 1200/900: com 1200 a frente e 900 de lado ele ficava
    # DENTRO do corredor por onde o portador precisa empurrar, e o chute caiu
    # de 5525 para 1536 mm/s. Medido duas vezes, revertido duas vezes.
    #
    # VIA OBSTRUIDA: recua para ATRAS da bola (APOIO_RECUO), onde a linha
    # costuma estar limpa porque a marcacao vem de frente. A constante e o
    # parametro 'bloqueado' existiam desde sempre e NUNCA eram usados - o
    # apoio ficava se oferecendo num ponto sem linha, e 'alvo_do_chute' nao
    # tinha para quem apontar.
    lado = 1.0 if (ordem % 2 == 0) else -1.0
    if bloqueado:
        return posicionamento.recuo_de_apoio(
            bx, by, _uax, _uay, _pax, _pay, lado) + (False,)
    return posicionamento.oferta_de_passe(
        bx, by, _uax, _uay, _pax, _pay, lado) + (False,)


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
    _gravar_papeis(papeis)

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
            # quem esta atras de nos ou alem do alvo nao atrapalha o tiro
            _folga = geometria.folga_lateral(
                tt.ball.position_x, tt.ball.position_y, _ux_c, _uy_c, _n,
                tt.enemy_robots)
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
    ordem = {PAPEL_PORTADOR: 0, PAPEL_APOIO: 0, PAPEL_COBERTURA: 0}
    for rid in sorted(tt.ally_robots):
        if rid == 0:
            continue
        papel = papeis.get(rid, PAPEL_COBERTURA)
        alvo_robo, tipo_robo = alvo_chute, tipo_alvo
        if papel == PAPEL_COBERTURA:
            alvo_robo, tipo_robo = None, "bloqueado"
        o = ordem.get(papel, 0)
        alvo_x, alvo_y, _ = alvo_do_papel(
            papel, situacao, rid, tt.ally_robots, tt.ball,
            gol_ataque, nosso_gol, ordem=o, alvo_chute=alvo_robo,
            inimigos=tt.enemy_robots, bloqueado=(tipo_robo == "bloqueado"))
        alvo_x, alvo_y = posicionamento.fora_da_area_penal(alvo_x, alvo_y, nosso_gol)
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
        if papel != PAPEL_COBERTURA and _dperto < RAIO_ORIENTA_CHUTE:
            _mira = alvo_robo or (gol_ataque.x, gol_ataque.y)
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
        if papel == PAPEL_COBERTURA:
            cmd.defensive_half = 1 if tt.on_positive_half else -1
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
        forca = chute.forca_por_alvo(tipo_robo)
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
                  % (rid, papel, d_bola, tipo_robo), flush=True)
        # O PORTAO DE CHUTE VIVE NA CAMADA DE SKILLS: skills/chute.py.
        #
        # Lá estão as tres licoes que a bola parada pagou e o jogo corrido teve
        # de redescobrir: a trava de armamento (a janela do grSim dura UMA
        # amostra), a guarda de "bola a frente da placa" (o teste do grSim e
        # simetrico e arma com a bola nas costas) e nao exigir alinhamento fino
        # no instante do contato.
        _sentido_x = -1.0 if tt.on_positive_half else 1.0
        _travas = tt.estado.setdefault("chute_armado", {}) \
            if hasattr(tt, "estado") and tt.estado is not None else {}
        if papel == PAPEL_COBERTURA:
            _travas.pop(rid, None)
            arma = False
        else:
            arma = chute.armar_chute(r, tt.ball, alvo_robo, _sentido_x,
                                     tipo_robo, _travas, rid)
        if os.environ.get("DIAG_JOGO"):
            # GEOMETRIA NO REFERENCIAL DO ROBO, que e o que o grSim arbitra:
            # ele dispara com 0 <= xx < 31,5 mm (placa) e |yy| < 40 mm.
            # Ver robot.cpp:120-128 e skills/chute.py. Sem isto nao da para
            # saber SE a bola chega na placa - so que o robo esta "perto".
            _xx, _yy, _ = chute.geometria_do_chutador(r, tt.ball)
            _frente = (alvo_robo is not None
                       and chute.direcao_para_frente(tt.ball, alvo_robo,
                                                     _sentido_x))
            print("[JG] PLACA r%d papel=%s xx=%.0f yy=%.0f arma=%s dispara=%s"
                  % (rid, papel, _xx, _yy, arma,
                     chute.na_janela_de_disparo(_xx, _yy)), flush=True)
            print("[JG] arma=%s frente=%s d=%.0f tipo=%s"
                  % (arma, _frente, d_bola, tipo_robo), flush=True)
        cmd.kick = forca if arma else 0.0
        # A cobertura nao disputa a bola: protege a abertura do gol.
        cmd.ball = papel != PAPEL_PORTADOR
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
