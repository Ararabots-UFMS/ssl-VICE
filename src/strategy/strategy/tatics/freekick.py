from new_movement.entities.States import Vector2D
from strategy.skills.skills import Skills
from strategy.behaviour import TaskStatus
from math import atan2, hypot, cos, sin, pi
import time


# ==========================================================================
#  Medidas do campo  (P3 / P14)
# ==========================================================================
class MedidasCampo:
    """Dimensoes da area de jogo, em milimetros.

    Padrao: Division B (regras SSL, secao 2.1.1 - area de jogo 9000 x 6000),
    logo linha de gol em x = +/-4500 e linha lateral em y = +/-3000.

    Antes esses numeros estavam escritos na mao como 2250/1500, que sao as
    medidas de um campo 4500 x 3000 (SSL-EL). Num campo de Division B isso
    mandava o goleiro para o meio-campo e fazia a validacao de posicao rejeitar
    quase todo o campo.

    Sempre que a geometria chegar da visao, ela tem prioridade sobre o padrao.
    """

    def __init__(self, comprimento: float = 9000.0, largura: float = 6000.0):
        self.comprimento = float(comprimento)
        self.largura = float(largura)

    @property
    def gol_x(self) -> float:
        return self.comprimento / 2.0

    @property
    def meia_largura(self) -> float:
        return self.largura / 2.0

    @classmethod
    def da_geometria(cls, geometry):
        """Le field_length/field_width da VisionGeometry; cai no padrao se faltar."""
        try:
            comprimento = float(getattr(geometry, "field_length", 0) or 0)
            largura = float(getattr(geometry, "field_width", 0) or 0)
            if comprimento > 0 and largura > 0:
                return cls(comprimento, largura)
        except (TypeError, ValueError):
            pass
        return cls()


class CenterGoal:
    """Centro dos gols. Mantido pelo nome antigo, agora derivado das medidas."""

    GOAL_POSITIVE = Vector2D(MedidasCampo().gol_x, 0.0)
    GOAL_NEGATIVE = Vector2D(-MedidasCampo().gol_x, 0.0)

    def __init__(self, medidas: MedidasCampo = None):
        medidas = medidas or MedidasCampo()
        self.GOAL_POSITIVE = Vector2D(medidas.gol_x, 0.0)
        self.GOAL_NEGATIVE = Vector2D(-medidas.gol_x, 0.0)


# ==========================================================================
#  Constantes de jogo
# ==========================================================================
RAIO_ROBO = 90.0            # regra 2.1.3: raio assumido de 0,09 m
DIST_TOQUE = 300.0          # a partir daqui o cobrador passa a empurrar a bola

# Ponto de aproximacao, medido do CENTRO da bola ao CENTRO do robo.
#
# Precisa ser MENOR que DIST_TOQUE, senao o cobrador chega ao ponto de
# aproximacao e para ali para sempre: a transicao para o empurrao nunca dispara e
# _can_kick nunca chega a ser consultado. Com 250 < 300 o robo entra na faixa de
# toque assim que estaciona, e a jogada avanca para o chute.
DIST_ATRAS_DA_BOLA = 250.0
# 250 mm: valor que FUNCIONOU na medicao de referencia (chute ativado, bola de
# (2500,0) para (3564,0)). Cheguei a subir para 400 achando que o robo precisava
# de espaco para alinhar - mas com o ponto mais longe ele deixou de chegar na
# bola dentro do tempo do teste. Nao mexer sem medir de novo.
# Medimos ele chegando a 109 mm da bola ainda com 86 mm de desvio lateral -
# dentro do alcance do chutador, mas torto. Como a trava de alinhamento barrava
# o empurrao, ele seguia em modo "aproximar", esbarrava de lado e jogava a bola
# para fora da linha. Com o ponto de aproximacao mais longe, qualquer excesso na
# frenagem ainda para ANTES da bola, e ha tempo de girar para a mira certa.
DIST_NO_PONTO = 150.0       # tolerancia para considerar que chegou ao ponto

# Desvio lateral maximo em relacao a linha bola->gol para liberar o empurrao.
# O toque fisico acontece por volta de 111 mm entre centros (raio do robo 90 +
# raio da bola 21); com 60 mm de folga o robo pega a bola cheia, e nao de raspao.
DESVIO_LATERAL_MAX = 80.0
# 80 mm: tambem o valor da medicao de referencia. Apertei para 50 mm por causa da
# largura da placa do chutador (40 mm), mas o controlador nao entrega essa
# precisao - a trava passava a barrar o empurrao e o robo esbarrava de lado.
# |lateral| < KickerWidth/2 = 40 mm). Com folga de 50 mm no gate, o contato cai
# dentro da placa em vez de raspar na quina do chassi.

# Distancia maxima ate a bola para LIBERAR o chute.
#
# O contato fisico acontece por volta de 111 mm entre centros (raio do robo 90 +
# raio da bola 21). No grSim o chutador so atua com a bola DENTRO da zona dele:
# se o comando chega antes disso, nada acontece e sobra o empurrao. Liberar o
# chute a 300 mm (a mesma faixa usada para decidir empurrar) era cedo demais -
# nos testes do Felipe a bola andava 287 mm por empurrao em vez de ser chutada.
DIST_CHUTE = 300.0

DIST_MIN_ADVERSARIO = 500.0 # regra 5.3.3: 0,5 m da bola na falta do adversario
VEL_MAX_BOLA = 6.5          # regra 8.4.2: velocidade maxima da bola, em m/s
# Tolerancia de mira: APERTADA de proposito.
#
# grSim robot.cpp:157-166 -> a bola sai na direcao do CORPO do robo
# (getBodyDirection), nao na direcao que a tatica calculou. Entao o erro de
# orientacao vira erro lateral no gol: a 2 m de distancia, 0.35 rad (20 graus)
# joga a bola 730 mm para o lado - e a meia-largura do gol e 500 mm.
# Com 0.15 rad (~8.6 graus) o desvio cai para ~300 mm e a bola entra.
TOLERANCIA_MIRA = 0.15

# Raio para considerar um robo como obstaculo.
#
# Declarar TODOS os robos como obstaculo, a cada ciclo e para cada robo, deixou
# update_obstacles caro demais: o driver passou a estourar o tempo de resposta
# dos servicos ("failed to send response (timeout)") e parte dos comandos se
# perdia. Como so importa desviar de quem esta no caminho, filtramos por
# proximidade - o custo cai e o desvio util continua.
RAIO_OBSTACULO = 1500.0

# Raio do contorno na aproximacao da falta.
#
# Numa cobranca o robo NAO pode simplesmente ir pela reta mais curta ate atras da
# bola: o planejador generico corta na frente dela e o robo acaba entre a bola e
# o gol. Medimos exatamente isso - robo em (2121,124) com a bola em (1844,265),
# 166 graus fora, empurrando a bola para o NOSSO campo.
#
# Por isso a aproximacao tem tres fases: contornar por fora, encaixar atras e so
# entao empurrar.
RAIO_CONTORNO = 700.0
MARGEM_ATRAS = 80.0     # o quanto o robo precisa estar atras da bola

# Desvio lateral tolerado DEPOIS que o empurrao ja comecou.
#
# E maior que DESVIO_LATERAL_MAX (80 mm, exigido para ENTRAR na fase) de
# proposito: entrar no empurrao pede alinhamento fino, mas cancelar o empurrao no
# meio por causa de 20 mm de deriva devolve o robo a oscilacao que a trava de
# fase existe para evitar. Quem decide se o chute sai continua sendo
# _atras_da_bola (10 graus) e _mira_esta_boa (0,15 rad).
DESVIO_TRAVADO = 150.0

# Tamanho maximo do passo pedido ao driver, em milimetros.
#
# POR QUE ISTO EXISTE - a medicao que motivou (sonda_saida.py)
# -----------------------------------------------------------
# Pedindo ao driver um alvo distante (robo em (1900,150), alvo (2250,0), erro de
# 381 mm), medimos a cadeia inteira:
#
#   - control.py comandou 0,876 m/s de forma CONSTANTE por 5,3 s;
#   - a velocidade REAL do robo ficou em 0 durante esses 5,3 s;
#   - quando ele finalmente reagiu, passou de 0 a 1120 mm/s e parou
#     em (3417,-1174) - 1,5 METRO alem do alvo, com o controlador ja saturado
#     em (-1,5 , +1,5) tentando traze-lo de volta.
#
# Sao duas falhas somadas, ambas fora de src/strategy/:
#   1. driver.py:publish_control avanca 'time_offset += dt' pelo relogio, sem
#      nenhuma realimentacao: a trajetoria termina no setpoint mesmo que o robo
#      nao tenha saido do lugar. Depois disso, replan() planeja a partir de
#      trajectory.get_state(time_offset) - o estado PLANEJADO, nunca o medido -
#      e todo novo pedido vira uma trajetoria de comprimento zero.
#   2. control.py so tem termo proporcional util (kp=2,3, ki=0) e limite de
#      1,5 m/s, sem perfil de frenagem: erro grande = comando grande, e nada
#      desacelera o robo na chegada.
#
# CONTORNO, todo dentro da estrategia: nunca pedir um alvo distante. A cada
# ciclo pedimos um passo curto medido A PARTIR DA POSICAO REAL do robo. Assim o
# erro visto pelo controlador fica sempre pequeno e limitado, o comando nunca
# satura, o setpoint nao consegue disparar na frente do robo (ele e recalculado
# da posicao medida a cada ciclo) e a ancora fantasma do driver se corrige
# sozinha - e o mesmo mecanismo do assentamento sob HALT (ver HANDOVER, secao 7),
# so que aplicado continuamente em vez de uma vez.
#
# Uma cobranca de falta nao precisa de velocidade, precisa de precisao: trocar
# alcance por controle e exatamente o negocio certo aqui.
# MODO MECANICO: um alvo por fase, sem picotar o caminho.
#
# O LACO QUE ISTO QUEBRA (medido em 24/08):
#
#   driver.replan()  ->  time_offset = 0.0   (driver.py:331 - toda chamada
#                                             reinicia a trajetoria do zero)
#   strategy.py      ->  so reenvia se o alvo mudou mais que 30 mm
#                        (LIMIAR_ALVO_MM, criado justamente para conter isso)
#   _passo_ate       ->  recalcula o alvo a partir da posicao MEDIDA, todo
#                        ciclo, com passo minimo de 60 mm
#
# Como o passo minimo (60) e maior que o limiar (30), o alvo mudava SEMPRE.
# Entao o driver replanejava sempre, e o time_offset voltava a zero sempre: o
# robo nunca executava mais que os primeiros ~100 ms de trajetoria nenhuma.
#
# Isso tambem explica a bimodalidade do erro de rastreio que nos confundiu: nas
# execucoes em que ele dava 7-22 mm, era o setpoint GRUDADO no robo, nao o robo
# seguindo bem. O robo ficava parado e o alvo ficava em cima dele.
#
# O passo curto foi criado para contornar o driver (§16.1). Medimos agora que
# ele causa um problema maior que o que resolve.
#
# Com False, cada fase manda o alvo FINAL uma vez; como esse alvo e funcao da
# bola (parada numa cobranca), ele nao muda, o _alvo_mudou devolve False e a
# trajetoria roda inteira. E o "angula e vai reto".
# TENTATIVA MECANICA - implementada, medida e REVERTIDA (24/08/2026).
#
# A ideia (do Felipe): "angula, vai reto e chuta" - um alvo por fase, sem
# picotar o caminho. Ela esta CERTA, e o motivo de nao funcionar aqui nao e a
# ideia: e que a estrategia nao tem como executa-la.
#
# O laco, medido:
#   driver.replan() zera o time_offset (driver.py:331), entao toda chamada a
#   strategy_command reinicia a trajetoria do zero. E strategy.py so reenvia se
#   o alvo mudou mais de 30 mm (LIMIAR_ALVO_MM), defesa criada contra isso.
#
# As tres formas foram testadas, 3 execucoes cada:
#
#   reenviar todo ciclo  -> replan a cada ciclo, time_offset sempre zero: o robo
#                           nunca passa dos primeiros ~100 ms de trajetoria
#   enviar UMA vez       -> a trajetoria acaba e ninguem corrige. Medido: bola
#                           andou 0 em 3 de 3, e numa delas o rastreio deu 4 mm
#                           com xx=467 - o robo PARADO com o setpoint em cima
#   segurar ~1 s         -> o valor recalculado e identico (o alvo e funcao da
#                           bola, que esta parada), entao _alvo_mudou bloqueia e
#                           vira o caso "uma vez". Bola andou 0 em 3 de 3
#
# Prova direta, da ultima execucao: o driver publicou UM alvo distinto em 1506
# publicacoes, amplitude 0 mm, em cima da posicao do robo; o ponto de encaixe
# ficou a 334 mm e nunca foi comandado.
#
# CONCLUSAO: a estrategia precisa reenviar para progredir, e reenviar destroi o
# progresso. Nao ha saida deste lado. A ideia mecanica tem de ser implementada
# na camada de movimento - um modo "va em linha reta ate este ponto", sem
# planner e sem time_offset, com o controlador fechando a malha na posicao
# medida. E o alvo da branch de refatoracao.
#
PASSO_MAX = 150.0

# Passo curto so funciona se o robo REAGIR a ele. Medindo a cadeia, ha execucoes
# em que ele fica parado apesar de haver comando: com 150 mm de erro o
# controlador pede 2,3 * 0,15 = 0,345 m/s, e abaixo de certo valor o robo
# simplesmente nao sai do lugar. Em 4 execucoes, 2 nao chegaram a chutar por
# causa disso.
#
# Entao o passo CRESCE enquanto ele estiver empacado, ate PASSO_EMPACADO, e
# volta ao normal assim que ele anda. E o minimo de autoridade para vencer a
# zona morta, sem abrir mao do controle quando ele esta obedecendo.
PASSO_EMPACADO = 450.0
VEL_EMPACADO = 30.0        # mm/s: abaixo disso, consideramos que nao saiu do lugar
CICLOS_EMPACADO = 5        # ~0,5 s a 10 Hz antes de comecar a aumentar

# FRENAGEM: o passo encolhe conforme o alvo se aproxima.
#
# POR QUE - control.py so tem termo proporcional util (kp=2,3) e limite de
# 1,5 m/s; nao existe perfil de desaceleracao em lugar nenhum da cadeia. Medimos
# o robo passar 1,5 METRO alem do alvo numa ida so, e nas execucoes perdidas ele
# chegava ao ponto de encaixe e seguia direto, indo parar em (2539,-178) e
# (2592,-153) - alem da bola e fora da linha, sem nunca chutar.
#
# Pedindo um passo proporcional ao que falta, o comando de velocidade cai junto
# com a distancia e o robo desacelera na chegada. E o perfil de frenagem que
# falta no controlador, feito pelo unico lado que podemos tocar.
#
# O piso existe para ele nao morrer na aproximacao final: precisa continuar
# pressionando ate fechar os ~104 mm que o chutador exige.
FRACAO_FREIO = 0.5
PASSO_MIN = 60.0

# Quanto o alvo do empurrao passa ALEM da bola, em milimetros.
# Ver _empurrar_para_o_gol para a medicao (chutador exige <= ~104 mm entre
# centros; parando o alvo na bola o robo estacionava a 107-116 mm).
# Quanto o alvo do empurrao pode passar ALEM da bola, em milimetros.
#
# ZERO, por medicao. A ideia de mandar o robo 150 mm alem da bola era faze-lo
# continuar pressionando ate fechar os ~104 mm que o chutador do grSim exige.
# Na pratica piorou: em 4 execucoes com 150 mm, tres viraram empurrao (bola
# andou 80, 259 e 309 mm). Com o alvo parando NA bola, o robo desacelera ao
# chegar e da ao chutador o instante parado de que ele precisa - medimos a bola
# saindo a ~5,5 m/s (498 mm em 0,09 s), que e chute de verdade.
# Quanto o alvo do empurrao fica ALEM da bola. Ele nunca e alcancado: existe
# para o robo seguir reto e com velocidade util ate o contato acontecer.
AVANCO_RETO = 350.0

# (LATERAL_ATE_S / LATERAL_DESLOC_MAX removidas - ver _empurrar_para_o_gol:
#  as duas versoes da correcao lateral foram testadas e revertidas.)

# (XX_PARA_SEGURAR foi removida - ver _empurrar_para_o_gol. Freava o robo em
# xx < 60 mm quando o grSim so dispara com xx < 31,5: ele parava na faixa onde
# chutar e impossivel.)

# Ate onde abrir a mira dentro da meta, em milimetros a partir do centro.
#
# POR QUE MIRAR FORA DO CENTRO - a medicao que motivou
# ----------------------------------------------------
# Com a mira no centro do gol os chutes ficaram PERFEITOS de direcao (desvio
# final em y entre -57 e -9 mm) e mesmo assim nao entravam: a bola parava em
# x=4198 e x=4206. O goleiro adversario estava em (4300, 0), e a menor distancia
# entre bola e goleiro nessas execucoes foi de 8 mm e 60 mm - o contato acontece
# a 111 mm (raio do robo 90 + raio da bola 21,5). Ou seja: eram DEFESAS, nao
# erros de pontaria. A unica execucao que fez gol foi a que passou a 179 mm dele.
#
# Meia-largura da meta = 500 mm. Mirando a 350 mm do centro sobram 150 mm ate o
# poste e a bola passa a 350 mm de um goleiro centrado - bem acima dos 111 mm de
# contato.
#
# Isto e o que distingue uma cobranca de falta de um chute qualquer: escolher o
# canto que o goleiro nao cobre.
MIRA_NO_CANTO = 350.0

# Ate que distancia da meta um adversario conta como goleiro, em milimetros.
RAIO_GOLEIRO = 1200.0

# Faixa morta para trocar de lado na mira, em milimetros.
#
# POR QUE - o goleiro do cenario fica em y=0, exatamente em cima da fronteira
# entre "esta em cima" e "esta embaixo". O ruido da visao o faz atravessar essa
# fronteira a cada quadro, e sem histerese o lado da mira ALTERNAVA entre -350 e
# +350: a linha bola->mira virava de lado, o ponto de encaixe pulava junto e o
# cobrador perseguia um alvo que trocava de lugar. Medido em duas execucoes
# perdidas - o robo passou da bola e foi parar em (2518,-350) e (2573,-358),
# com 341 mm de desvio da linha, sem nunca chegar a chutar.
#
# Com a trava, o lado e escolhido UMA vez e so muda se o goleiro se mudar de
# verdade para o lado que estamos mirando, ultrapassando esta faixa.
HISTERESE_MIRA = 150.0

# ==========================================================================
#  PASSE  -  quando nao da para chutar a gol
# ==========================================================================
#
# POR QUE ISTO PASSOU A EXISTIR
# -----------------------------
# Ate agora, quando _can_kick() era falso (bola longe demais do gol adversario),
# a cobranca simplesmente NAO armava o chute: o cobrador ficava empurrando a
# bola para a frente indefinidamente ate a jogada expirar. Numa falta de campo
# de defesa isso e simplesmente perder a posse.
#
# A regra e a de qualquer cobranca: se da para finalizar, finaliza; se nao da,
# toca para quem esta melhor colocado. _posicao_de_apoio ja colocava os
# companheiros numa linha de passe - so faltava alguem passar para eles.
#
# CRITERIOS PARA UM COMPANHEIRO SER ALVO DE PASSE:
#   - nao e o proprio cobrador nem o goleiro;
#   - esta ADIANTE da bola no sentido do ataque (>= AVANCO_MIN_PASSE);
#   - esta a uma distancia util (nem colado, nem longe demais para a bola
#     chegar - ver a medicao de alcance abaixo);
#   - a linha ate ele esta livre de adversarios (FOLGA_LINHA_PASSE).
# Entre os que passam nos criterios, escolhemos o mais adiantado.
AVANCO_MIN_PASSE = 400.0     # o companheiro precisa estar a frente da bola
DIST_MIN_PASSE = 700.0       # abaixo disso nao vale a pena passar
DIST_MAX_PASSE = 3000.0      # acima disso a bola nao chega (ver alcance medido)
FOLGA_LINHA_PASSE = 400.0    # distancia minima de um adversario ate a linha

# O quanto o companheiro pode estar ATRAS da bola e ainda valer o passe.
#
# Negativo de proposito: quando nao ha ninguem a frente, tocar para tras e para
# o lado e a jogada certa. Antes disso o cobrador ficava empurrando a bola
# sozinho ate a jogada expirar - perder a posse era o comportamento padrao numa
# falta encurralada. A ordem de preferencia esta em _alvo_da_jogada.
RECUO_MAX_PASSE = -2500.0

# Folga exigida na linha de TIRO, bem menor que a do passe.
#
# Sao coisas diferentes e eu confundi as duas, com efeito devastador: usando os
# 400 mm do passe, o goleiro adversario em (4300,0) fica a 310 mm da linha de
# tiro para o canto e a jogada era declarada "bloqueada" - o chute parou de sair
# em 4 de 4 execucoes do cenario que antes fazia gol.
#
# Num passe queremos um corredor limpo, porque a bola vai devagar e da tempo de
# interceptar. Num chute a gol queremos exatamente o contrario: passar RENTE ao
# goleiro. O limite fisico e o contato, raio do robo 90 + raio da bola 21,5 =
# 111,5 mm; a execucao que fez gol passou a 179 mm dele. 200 mm e o minimo com
# algum respiro, sem descartar o unico angulo que funciona.
FOLGA_LINHA_TIRO = 200.0

# ==========================================================================
#  GEOMETRIA DO CHUTADOR DO grSim  (robot.cpp:120-128)
# ==========================================================================
#
# O grSim so dispara o chute se, NO INSTANTE do comando, a bola estiver:
#     xx < KickerThickness*2 + BallRadius = 31,5 mm   da face da placa
#     yy < KickerWidth/2 = 40 mm                      do eixo do corpo
# com a placa a CenterFromKicker = 73 mm do centro do robo.
#
# POR QUE ISTO VIROU UMA TRAVA EXPLICITA
# --------------------------------------
# O sintoma relatado - "ele sempre fura ou empurra a bola, nunca chuta" - vem
# de duas coisas que se somam:
#
#   1. A janela de contato e estreitissima. Medindo ao vivo, xx caiu de 31,8
#      para 30,2 mm e ficou dentro do limite por UMA amostra antes de a bola
#      partir. Se o chute nao estiver armado exatamente nesse instante, o que
#      sobra e o empurrao do corpo - foi o que medimos: bola saindo a
#      1,2-3,4 m/s em vez dos 6,4 m/s do chute.
#
#   2. As travas antigas (_atras_da_bola, _mira_esta_boa, _movimento_ok...)
#      piscavam durante a aproximacao, ligando e desligando o chute. Como
#      control.py guarda o ultimo valor recebido (kick_cache), bastava um ciclo
#      com kick=0 na hora errada para o toque virar empurrao.
#
# Solucao: medir a MESMA geometria que o grSim mede, e ARMAR o chute assim que o
# alinhamento lateral estiver bom - mantendo armado enquanto continuar bom. Nao
# adianta so estar "atras da bola": o que decide e o eixo do corpo passar pela
# bola dentro de 40 mm.
CENTRO_ATE_PLACA = 73.0
ESPESSURA_PLACA = 5.0
RAIO_BOLA_MM = 21.5
LIM_XX_GRSIM = ESPESSURA_PLACA * 2.0 + RAIO_BOLA_MM     # 31,5 mm
LIM_YY_GRSIM = 40.0

# Para ARMAR exigimos folga sobre o limite do grSim; para MANTER armado
# aceitamos ate o limite. Sem essa histerese o chute volta a piscar.
YY_PARA_ARMAR = 32.0
# 25 -> 32, com medicao. Refazendo o teste do grSim (robot.cpp:128) quadro a
# quadro na visao CRUA, a janela do chutador abriu em 4 de 6 execucoes - e em 2
# delas ficou aberta por ~30 quadros (0,5 s) com a estrategia NUNCA armada.
# Perdiamos o chute com a geometria na mao.
#
# A causa e aritmetica, do mesmo tipo do XX_PARA_SEGURAR: exigiamos yy <= 25
# para ARMAR quando o simulador aceita ate 40. O HANDOVER §23 ja lista isto
# entre os erros proprios ("fiz nossa trava mais rigida que a do simulador").
#
# 32 mantem 20% de folga sobre os 40 e fica abaixo dos 38 que o §18 registra
# como geradores de tiro de raspao - e aquele teste rodou com o XX_PARA_SEGURAR
# ainda no lugar, quando o robo parava a 50-60 mm e todo contato era marginal.
# A histerese continua: arma em 32, sustenta ate 40.
YY_PARA_MANTER = LIM_YY_GRSIM
# Distancia da placa a partir da qual ja vale deixar o chute armado esperando o
# contato. Armar cedo nao tem custo: o grSim so dispara quando encosta.
XX_PARA_ARMAR = 250.0

# Alcance da bola em BAIXA velocidade, medido com atrito_bola.py:
#
#     1,0 m/s ->  379 mm   (desaceleracao 1,32 m/s2)
#     1,5 m/s ->  838 mm   (desaceleracao 1,34 m/s2)
#     2,5 m/s -> 2321 mm   (desaceleracao 1,35 m/s2)
#
# A desaceleracao e constante em ~1,33 m/s2 nessa faixa (bem diferente dos
# ~9 m/s2 do chute forte, em que a bola desliza antes de rolar). Disso sai
# v = sqrt(2 * 1,33 * d), ou seja v ~ 1,64 * sqrt(d) para a bola PARAR em cima
# do companheiro. Usamos 1,9 para ela chegar ainda rolando, que e o que se quer
# num passe - o companheiro recebe a bola viva, nao parada.
FATOR_PASSE = 1.9
VEL_MIN_PASSE = 1.5


class EstadoFreekick:
    """Memoria da jogada, que sobrevive entre os ciclos.  (P7 / P8)

    Antes o codigo tentava ler isso de um 'parent' da arvore de comportamento -
    mas behaviour.py nao tem get_parent nem _parent. O resultado era last_kicker_id
    sempre None: a protecao de duplo toque nunca funcionava e o cronometro nunca
    contava. Agora a jogada guarda o proprio estado e o injeta nas acoes.
    """

    def __init__(self):
        self.last_kicker_id = None
        self.last_kick_time = None
        self.inicio = None
        # Fase TRAVADA de cada cobrador (robot_id -> 'empurrar').
        #
        # Sem isso a fase era recalculada do zero a cada ciclo, e a cobranca
        # entrava em oscilacao: '_pronto_para_empurrar' exige o robo parado
        # (_robot_is_stable, < 100 mm/s), entao no instante em que ele obedecia e
        # comecava a empurrar deixava de estar parado - e a fase voltava para
        # 'encaixar', cujo alvo fica ATRAS dele.
        #
        # Medido com a sonda deterministica (sonda_ciclo.py): 33 trocas de fase
        # em 61 ciclos, com o alvo comandado alternando entre a bola (2500) e o
        # ponto de encaixe (2250) a cada ciclo. Como os dois alvos distam 250 mm,
        # o filtro de reenvio de strategy.py (30 mm) deixa passar TODOS eles: o
        # driver recebe um alvo contraditorio a 30 Hz e nunca chega a executar
        # uma trajetoria inteira. Era essa a origem da dispersao entre execucoes
        # identicas - ela dependia so de em qual fase o ciclo calhava de estar
        # quando o robo alcancava a bola.
        self.fase_travada = {}
        # Ciclos seguidos com o robo parado apesar de ter para onde ir.
        # Ver _passo_atual: serve para vencer a zona morta do controlador.
        self.empacado = {}
        # Lado da meta em que estamos mirando (-1 ou +1), TRAVADO.
        # Ver _ponto_de_mira para a medicao que obrigou a travar.
        self.lado_mira = None
        # Companheiro escolhido para receber o passe, TRAVADO uma vez decidido.
        # Mesma licao de lado_mira: alvo que troca a cada ciclo faz o cobrador
        # perseguir um ponto que nunca para quieto.
        self.receptor = None
        # Chute ARMADO por robo. Ver o bloco da geometria do chutador: o disparo
        # so acontece se o chute estiver armado no instante exato do contato.
        self.chute_armado = {}

    def iniciar_se_preciso(self):
        if self.inicio is None:
            self.inicio = time.time()

    def reiniciar(self):
        self.inicio = None
        self.last_kicker_id = None
        self.last_kick_time = None
        self.fase_travada = {}
        self.empacado = {}
        self.lado_mira = None
        self.receptor = None
        self.chute_armado = {}

    def decorrido(self) -> float:
        return 0.0 if self.inicio is None else time.time() - self.inicio

    def registrar_chute(self, robot_id):
        self.last_kicker_id = robot_id
        self.last_kick_time = time.time()


class GoalkeeperKickoff:
    """Leva o goleiro para o centro da propria meta."""

    def __init__(self):
        self.name = "GoalkeeperKickoff"
        self.skills_factory = Skills("Movement")

    def execute(self, goal_position: Vector2D, angle: float, ally_ids=None,
                enemy_ids=None):
        robot_command = self.skills_factory.move_with_angle(
            robot_id=0,
            target_x=goal_position.x,
            target_y=goal_position.y,
            vel_x=0.0,
            vel_y=0.0,
            angle=angle,
        )
        robot_command.field_border = True
        # P6: desviar dos outros robos tambem no trajeto ate a meta.
        robot_command.ally_ids = [r for r in (ally_ids or []) if r != 0]
        robot_command.enemy_ids = list(enemy_ids or [])
        robot_command.deactivate_kick()
        return robot_command


class _BaseFreekick:
    """Parte comum as duas jogadas: medidas, estado, obstaculos e parada segura."""

    def __init__(self, ally_robots, ball, on_positive_half, enemy_robots=None,
                 estado=None, medidas=None, last_kicker_id=None,
                 last_kick_time=None):
        self.skills_factory = Skills("Movement")
        self.ally_robots = ally_robots or {}
        self.enemy_robots = enemy_robots or {}
        self.ball = ball
        self.on_positive_half = on_positive_half

        self.medidas = medidas or MedidasCampo()
        self.goal_center = CenterGoal(self.medidas)

        # P1: aceitamos last_kick_time. A chamada em plays/freekick.py ja passava
        # esse argumento, mas o construtor nao o declarava - era TypeError a cada
        # ciclo, antes de qualquer logica rodar.
        self.estado = estado or EstadoFreekick()
        if last_kicker_id is not None:
            self.estado.last_kicker_id = last_kicker_id
        if last_kick_time is not None:
            self.estado.last_kick_time = last_kick_time

        self.max_execution_time = 30.0

        if self.on_positive_half:
            self.gk_angle = pi
            self.gk_target = self.goal_center.GOAL_POSITIVE
            self.attack_goal = self.goal_center.GOAL_NEGATIVE
            self.own_goal = self.goal_center.GOAL_POSITIVE
        else:
            self.gk_angle = 0.0
            self.gk_target = self.goal_center.GOAL_NEGATIVE
            self.attack_goal = self.goal_center.GOAL_POSITIVE
            self.own_goal = self.goal_center.GOAL_NEGATIVE

    # ---------------------------------------------------------------- apoio
    def _ids_de_linha(self):
        return sorted(r for r in self.ally_robots if r != 0)

    def _pos(self, robot_id):
        r = self.ally_robots.get(robot_id)
        return None if r is None else (r.position_x, r.position_y)

    def _dist_ate_bola(self, robot_id):
        p = self._pos(robot_id)
        if p is None:
            return float("inf")
        return hypot(p[0] - self.ball.position_x, p[1] - self.ball.position_y)

    def _dentro_do_campo(self, x, y):
        """Puxa o alvo para dentro das linhas, com folga de um raio de robo."""
        lim_x = self.medidas.gol_x - RAIO_ROBO
        lim_y = self.medidas.meia_largura - RAIO_ROBO
        return max(-lim_x, min(lim_x, x)), max(-lim_y, min(lim_y, y))

    def _proximos(self, robot_id, robots):
        """Ids de 'robots' a menos de RAIO_OBSTACULO do robo dado."""
        p = self._pos(robot_id)
        if p is None:
            return []
        perto = []
        for rid, r in robots.items():
            if rid == robot_id and robots is self.ally_robots:
                continue
            if hypot(r.position_x - p[0], r.position_y - p[1]) <= RAIO_OBSTACULO:
                perto.append(rid)
        return perto

    def _obstaculos(self, comando, robot_id, evitar_bola):
        """Declara como obstaculo o que esta por perto.

        Antes so field_border, penalty_area e ball eram marcados: sem enemy_ids e
        ally_ids o planejador nao desviava de robo nenhum, e o cobrador tentava
        atravessar a barreira adversaria.

        Declarar TODOS os robos, porem, custou caro demais - o driver passou a
        estourar o tempo de resposta dos servicos. Filtrar por proximidade
        preserva o desvio que importa (quem esta no caminho) com uma fracao do
        custo.
        """
        comando.field_border = True
        comando.penalty_area = True
        comando.ball = evitar_bola
        comando.ally_ids = self._proximos(robot_id, self.ally_robots)
        comando.enemy_ids = self._proximos(robot_id, self.enemy_robots)
        return comando

    def _check_timeout(self) -> bool:
        """P8: o cronometro pertence a jogada, nao ao executor.

        Antes start_time era gravado no __init__, e um executor novo nascia a cada
        ciclo - o tempo decorrido era sempre ~0 e o limite nunca era atingido.
        """
        self.estado.iniciar_se_preciso()
        return self.estado.decorrido() > self.max_execution_time

    def _check_double_touch(self, robot_id) -> bool:
        """Regra 8.2: quem cobrou nao pode tocar de novo antes de outro robo."""
        if self.estado.last_kicker_id is None:
            return True
        return robot_id != self.estado.last_kicker_id

    def _parada_segura(self, motivo=""):
        """P9: em vez de lista vazia, devolve posicoes seguras.

        Antes o timeout e a bola invalida faziam 'return []'. Sem comando novo os
        robos simplesmente MANTINHAM o ultimo alvo recebido - o sistema continuava
        executando uma ordem velha. Agora o goleiro vai para a meta e os demais se
        afastam da bola ate a distancia minima da regra.
        """
        comandos = []
        if 0 in self.ally_robots:
            comandos.append(
                GoalkeeperKickoff().execute(
                    self.gk_target, self.gk_angle,
                    ally_ids=list(self.ally_robots), enemy_ids=list(self.enemy_robots)
                )
            )
        for rid in self._ids_de_linha():
            p = self._pos(rid)
            if p is None:
                continue
            bx, by = self.ball.position_x, self.ball.position_y
            dx, dy = p[0] - bx, p[1] - by
            d = hypot(dx, dy) or 1.0
            folga = DIST_MIN_ADVERSARIO + RAIO_ROBO
            if d >= folga:
                alvo_x, alvo_y = p              # ja esta longe: fica onde esta
            else:
                alvo_x = bx + dx / d * folga
                alvo_y = by + dy / d * folga
            alvo_x, alvo_y = self._dentro_do_campo(alvo_x, alvo_y)
            cmd = self.skills_factory.move_with_angle(
                robot_id=rid, target_x=alvo_x, target_y=alvo_y,
                vel_x=0.0, vel_y=0.0, angle=atan2(by - alvo_y, bx - alvo_x),
            )
            self._obstaculos(cmd, rid, evitar_bola=True)
            cmd.deactivate_kick()
            comandos.append(cmd)
        return comandos


class OurFreekick(_BaseFreekick):
    """Nossa cobranca de falta."""

    def __init__(self, ally_robots, ball, on_positive_half, enemy_robots=None,
                 estado=None, medidas=None, last_kicker_id=None,
                 last_kick_time=None):
        super().__init__(ally_robots, ball, on_positive_half, enemy_robots,
                         estado, medidas, last_kicker_id, last_kick_time)
        self.name = "OurAtack"
        # metade da distancia entre o centro e o gol adversario
        self.kick_threshold = self.medidas.gol_x / 2.0

    # ------------------------------------------------------------ validacao
    def _is_ball_position_valid(self):
        """A bola esta num lugar em que faz sentido cobrar?

        As margens (200 mm da linha, 1000 mm da area) sao as mesmas de antes -
        so a escala mudou: eram fixas em 2250/1500, de um campo 4500 x 3000.
        """
        x, y = self.ball.position_x, self.ball.position_y
        if abs(x) > self.medidas.gol_x - 200 or abs(y) > self.medidas.meia_largura - 200:
            return False
        if abs(x) > self.medidas.gol_x - 1000 and abs(y) < 750:
            return False
        return True

    def _can_kick(self):
        """A bola esta avancada o bastante para valer um chute a gol?"""
        if self.on_positive_half:
            return self.ball.position_x < -self.kick_threshold
        return self.ball.position_x > self.kick_threshold

    def _geometria_do_chutador(self, robot_id):
        """(xx, yy) da bola em relacao a placa do chutador, como o grSim calcula.

        Reproduz robot.cpp:120-128. xx = distancia ao longo do eixo do corpo
        (quao perto a bola esta da face da placa), yy = desalinhamento lateral.
        """
        r = self.ally_robots.get(robot_id)
        if r is None:
            return None
        dx, dy = cos(float(r.orientation)), sin(float(r.orientation))
        kx = r.position_x + (CENTRO_ATE_PLACA + ESPESSURA_PLACA * 0.5) * dx
        ky = r.position_y + (CENTRO_ATE_PLACA + ESPESSURA_PLACA * 0.5) * dy
        ex = kx - self.ball.position_x
        ey = ky - self.ball.position_y
        return abs(ex * dx + ey * dy), abs(-ex * dy + ey * dx)

    def _chute_deve_estar_armado(self, robot_id) -> bool:
        """Manter o chute armado agora?

        Arma quando o corpo esta apontado para a bola dentro de YY_PARA_ARMAR e
        a placa ja esta chegando (XX_PARA_ARMAR); mantem armado ate o
        alinhamento passar de YY_PARA_MANTER. Ver o bloco da geometria do
        chutador para o motivo de isto existir.
        """
        g = self._geometria_do_chutador(robot_id)
        if g is None:
            return False
        xx, yy = g
        if self.estado.chute_armado.get(robot_id):
            # UMA VEZ ARMADO, SEGUE ARMADO enquanto a placa estiver chegando.
            #
            # Antes desarmavamos assim que o NOSSO yy passava de 40. Mas esse yy
            # vem da visao filtrada, e a orientacao dela atrasa: com o filtro
            # corrigido sao ~4,7 graus, sem ele 17-23 - o que vale 8 a 42 mm de
            # yy so de atraso. O resultado e o chute piscando durante a
            # aproximacao final, e a chance de estar armado justo no quadro do
            # contato virar sorte.
            #
            # MEDIDO refazendo o teste do grSim quadro a quadro na visao crua:
            # a janela do chutador abriu em 4 de 6 execucoes, ficou aberta ~30
            # quadros (0,5 s) em duas delas, e o disparo nao saiu - com a
            # estrategia tendo pedido chute em algum outro momento. Pedido e
            # janela simplesmente nao se encontravam.
            #
            # O HANDOVER §18 ja prescrevia isto: "arme o chute durante toda a
            # aproximacao final. Armar cedo nao custa nada: o grSim so dispara
            # no contato. Recusar armar, sim, custa."
            #
            # Quem arbitra continua sendo o grSim (robot.cpp:128 exige xx<31,5 E
            # yy<40 no instante do comando): manter armado nao cria chute torto,
            # so deixa de perder o chute certo. A direcao segue protegida por
            # _atras_da_bola (10 graus) e _mira_esta_boa (0,15 rad), que sao
            # verificadas antes de o comando sair.
            armado = xx <= XX_PARA_ARMAR
        else:
            armado = yy <= YY_PARA_ARMAR and xx <= XX_PARA_ARMAR
        self.estado.chute_armado[robot_id] = armado
        return armado

    def _armar_se_der(self, comando, robot_id):
        """Arma o chute em QUALQUER fase, se a geometria e a direcao permitirem.

        POR QUE ISTO EXISTE - medido em 24/08
        --------------------------------------
        O chute so era armado dentro da fase de EMPURRAO. Nas fases 'contornar'
        e 'encaixar' o codigo chamava deactivate_kick() incondicionalmente.

        Refazendo o teste do grSim quadro a quadro na visao crua, num lote de 12:

            rep     janela aberta   desses ARMADOS   disparou
            1            130              0            NAO
            2             28              0            NAO
            5             14              0            NAO
            7            118              0            NAO

        290 quadros de geometria perfeita - quase 5 segundos - com o chute
        armado em ZERO deles. Nao era azar de temporizacao: era impossivel por
        construcao, porque o robo nao estava na fase que arma.

        O HANDOVER §18 ja prescrevia o contrario: "arme o chute durante toda a
        aproximacao final. Armar cedo nao custa nada: o grSim so dispara no
        contato. Recusar armar, sim, custa."

        A TRAVA QUE NAO PODE SAIR: _mira_esta_boa. O teste do chutador do grSim
        e SIMETRICO - a bola alinhada ao eixo satisfaz o disparo com o robo de
        frente ou de costas (§18). Sem checar a direcao, ja medimos 6,4 m/s
        mandando a bola de (2500,0) para (1580,-523), o nosso campo.
        """
        if (self._tem_alvo_valido()
                and self._check_double_touch(robot_id)
                and self._mira_esta_boa(robot_id)
                and self._chute_deve_estar_armado(robot_id)):
            comando.kick = self._forca_do_chute(robot_id)
        else:
            self.estado.chute_armado[robot_id] = False
            comando.deactivate_kick()
        return comando

    def _mira_esta_boa(self, robot_id) -> bool:
        """P11: o robo esta mesmo apontado para o gol?

        Antes bastava a bola estar alem do limiar em x. O robo chutava de costas
        para o gol ou de lado, e a bola ia para qualquer direcao.
        """
        r = self.ally_robots.get(robot_id)
        if r is None:
            return False
        mx, my, _ = self._alvo_da_jogada()
        desejado = atan2(my - r.position_y, mx - r.position_x)
        erro = (float(r.orientation) - desejado + pi) % (2 * pi) - pi
        return abs(erro) <= TOLERANCIA_MIRA

    # Velocidade abaixo da qual consideramos o robo "assentado" no ponto de
    # aproximacao. 25 mm/s era rigido demais: o controlador fica micro-corrigindo
    # e o robo nunca atingia esse valor, entao a fase de empurrao nunca comecava
    # e ele acabava esbarrando na bola sem querer. 100 mm/s ja e praticamente
    # parado para o tamanho do campo.
    VEL_ASSENTADO = 100.0

    def _robot_is_stable(self, robot_id) -> bool:
        """O robo esta praticamente parado?"""
        r = self.ally_robots.get(robot_id)
        if r is None:
            return False
        return hypot(r.velocity_x, r.velocity_y) < self.VEL_ASSENTADO

    def _movimento_ok_para_chutar(self, robot_id) -> bool:
        """O robo esta em condicao de chutar com pontaria?

        Aceita dois casos:
          1) parado - mira firme, chute parado;
          2) em movimento NA DIRECAO do gol, com no maximo ~45 graus de desvio.

        Por que mudou: exigir apenas 'parado' (velocidade < 25 mm/s) anulava o
        proprio empurrao. Na fase de empurrao o cobrador esta, por definicao, se
        movendo para cima da bola - nunca ficava estavel, entao nunca chutava.
        Nos testes o chute nao ativou em nenhum dos 13 cenarios por causa disso.
        Um chute de verdade acontece com o robo em movimento; o que importa nao e
        estar parado, e estar indo para o lado certo.
        """
        r = self.ally_robots.get(robot_id)
        if r is None:
            return False

        vx, vy = float(r.velocity_x), float(r.velocity_y)
        velocidade = hypot(vx, vy)
        if velocidade < 25:
            return True                       # caso 1: parado

        ux, uy = self._versor_bola_gol()
        # cos do angulo entre a velocidade e a direcao bola->gol
        cos_ang = (vx * ux + vy * uy) / velocidade
        return cos_ang >= 0.7                 # ~45 graus

    def _forca_do_chute(self, robot_id) -> float:
        """P15: forca proporcional a distancia ate o gol, limitada pela regra.

        Antes era sempre 1.5, tanto para um toque curto quanto para finalizar de
        longe. O teto de 6,5 m/s vem da regra 8.4.2 (Ball Speed).
        """
        p = self._pos(robot_id)
        if p is None:
            return 3.0

        # PASSE: forca calculada para a bola CHEGAR ao companheiro ainda
        # rolando, nao para atravessar o campo. Ver FATOR_PASSE - em baixa
        # velocidade a desaceleracao medida e constante em ~1,33 m/s2, entao
        # v = 1,9 * sqrt(d) poe a bola no pe dele com sobra pequena.
        #
        # Chutar um passe com a forca de finalizacao (6,4 m/s) mandaria a bola
        # a mais de 2 metros alem do companheiro - seria entregar a posse.
        alvo_x, alvo_y, tipo = self._alvo_da_jogada(robot_id)
        if tipo == "passe":
            d_m = hypot(alvo_x - self.ball.position_x,
                        alvo_y - self.ball.position_y) / 1000.0
            return min(VEL_MAX_BOLA - 0.1,
                       max(VEL_MIN_PASSE, FATOR_PASSE * (d_m ** 0.5)))

# FINALIZACAO: sempre o maximo que a regra permite.
        #
        # O calculo proporcional a distancia foi abandonado, e o piso max(6.0,
        # ...) que sobrava dele era enganoso: a 2,3 m do gol ele dava
        # min(6,4 , max(6,0 , 3,5)) = 6,0 m/s, ou seja o teto de 6,4 NUNCA valia
        # numa falta tipica. Ficava parecendo que a forca tinha subido quando na
        # pratica nao tinha.
        #
        # Com o atrito medido neste grSim (alcance satura pouco acima de 2 m,
        # ver atrito_bola.py) nao existe distancia util em que valha a pena
        # chutar mais fraco a gol. Entao: o maximo, sempre.
        # O teto de 6,5 m/s e da regra 8.4.2; 0,1 e a margem de seguranca.
        return VEL_MAX_BOLA - 0.1

    # ------------------------------------------------------------- papeis
    def _eleger_cobrador(self):
        """P5: um unico cobrador - o mais proximo da bola.

        Antes TODO robo de linha recebia o mesmo alvo (a bola), entao dois ou tres
        disputavam a mesma coordenada, colidiam e se bloqueavam. O desempate por
        id evita que a escolha fique oscilando entre ciclos.
        """
        candidatos = self._ids_de_linha()
        if not candidatos:
            return None
        return min(candidatos, key=lambda r: (round(self._dist_ate_bola(r), 1), r))

    def _ponto_de_mira(self):
        """Onde mirar DENTRO da meta adversaria.

        Sem adversario perto da meta, o centro serve. Com goleiro, mira no canto
        oposto ao lado em que ele esta - ver MIRA_NO_CANTO para as medicoes que
        mostraram chutes perfeitos sendo defendidos por 8 mm.
        """
        gx = self.attack_goal.x
        goleiros = [r for r in self.enemy_robots.values()
                    if abs(r.position_x - gx) < RAIO_GOLEIRO]
        if not goleiros:
            return gx, 0.0
        gk = min(goleiros, key=lambda r: abs(r.position_x - gx))

        # Lado TRAVADO: so troca se o goleiro se mudar de verdade para o canto
        # que estamos mirando. Ver HISTERESE_MIRA.
        lado = self.estado.lado_mira
        if lado is None or gk.position_y * lado > HISTERESE_MIRA:
            lado = -1.0 if gk.position_y >= 0.0 else 1.0
            self.estado.lado_mira = lado
        return gx, lado * MIRA_NO_CANTO

    def _linha_livre(self, ax, ay, bx, by, folga=FOLGA_LINHA_PASSE) -> bool:
        """Nenhum adversario a menos de 'folga' do segmento a->b."""
        vx, vy = bx - ax, by - ay
        comp2 = vx * vx + vy * vy
        for r in self.enemy_robots.values():
            wx, wy = r.position_x - ax, r.position_y - ay
            t = 0.0 if comp2 <= 0 else max(0.0, min(1.0, (wx * vx + wy * vy) / comp2))
            if hypot(wx - t * vx, wy - t * vy) < folga:
                return False
        return True

    def _companheiro_para_passe(self, cobrador, avanco_min, com_goleiro=False):
        """Melhor companheiro para receber, ou None.

        'avanco_min' e o quanto ele precisa estar a frente da bola no sentido do
        ataque. Chamando com um valor negativo aceitamos passe lateral ou para
        TRAS - ver _alvo_da_jogada para a ordem de preferencia.
        """
        ux, uy = self._versor_bola_gol_puro()

        melhor = None
        for rid, r in self.ally_robots.items():
            if rid == cobrador:
                continue
            if rid == 0 and not com_goleiro:
                continue
            px = r.position_x - self.ball.position_x
            py = r.position_y - self.ball.position_y
            avanco = px * ux + py * uy
            dist = hypot(px, py)
            if avanco < avanco_min:
                continue
            if not (DIST_MIN_PASSE <= dist <= DIST_MAX_PASSE):
                continue
            if not self._linha_livre(self.ball.position_x, self.ball.position_y,
                                     r.position_x, r.position_y):
                continue
            # entre os validos, o mais adiantado
            if melhor is None or avanco > melhor[0]:
                melhor = (avanco, rid)
        return None if melhor is None else melhor[1]

    def _alvo_da_jogada(self, cobrador=None):
        """Para onde esta cobranca vai: o gol, ou um companheiro.

        Devolve (x, y, tipo) com tipo em {"gol", "passe"}.

        Se da para finalizar (_can_kick), finaliza. Se nao da, procura quem
        esta melhor colocado e toca para ele - ver o bloco de constantes do
        PASSE. Sem ninguem disponivel, mantem a direcao do gol: e o
        comportamento antigo, e pelo menos leva a bola para a frente.
        """
        if cobrador is None:
            cobrador = self._eleger_cobrador()

        # 1) FINALIZAR - so se o gol esta ao alcance E a linha de tiro esta livre.
        #    Checar a linha aqui e o que impede o cobrador de chutar contra as
        #    costas de um adversario parado na frente, que era o comportamento
        #    antigo (so a distancia era verificada).
        if self._can_kick():
            mx, my = self._ponto_de_mira()
            if self._linha_livre(self.ball.position_x, self.ball.position_y,
                                 mx, my, FOLGA_LINHA_TIRO):
                return mx, my, "gol"

        # 2) Receptor ja escolhido continua valendo enquanto a linha estiver
        #    livre. Alvo que troca a cada ciclo faz o cobrador perseguir um ponto
        #    que nunca para quieto - mesma licao de lado_mira.
        rid = self.estado.receptor
        if rid is not None and rid in self.ally_robots and rid != cobrador:
            r = self.ally_robots[rid]
            if self._linha_livre(self.ball.position_x, self.ball.position_y,
                                 r.position_x, r.position_y):
                return r.position_x, r.position_y, "passe"

        # 3) PASSE PARA A FRENTE - o melhor caso, ganha terreno.
        # 4) PASSE LATERAL OU PARA TRAS - pior que avancar, mas MUITO melhor que
        #    ficar empurrando a bola sozinho ate a jogada expirar, que era o que
        #    acontecia antes. Uma falta encurralada se resolve recuando a bola
        #    para quem tem campo pela frente.
        # 5) Em ultimo caso o proprio goleiro, que quase sempre esta livre.
        for avanco_min, com_gk in ((AVANCO_MIN_PASSE, False),
                                   (RECUO_MAX_PASSE, False),
                                   (RECUO_MAX_PASSE, True)):
            rid = self._companheiro_para_passe(cobrador, avanco_min, com_gk)
            if rid is not None:
                self.estado.receptor = rid
                r = self.ally_robots[rid]
                return r.position_x, r.position_y, "passe"

        # 6) Ninguem disponivel: mantem a direcao do gol. Pelo menos leva a bola
        #    para a frente, e e o comportamento historico.
        self.estado.receptor = None
        mx, my = self._ponto_de_mira()
        return mx, my, "gol"

    def _tem_alvo_valido(self) -> bool:
        """Ha para onde mandar a bola de verdade?

        Precisa refletir a MESMA decisao de _alvo_da_jogada. Antes isto era
        'passe ou _can_kick()', e o _can_kick sozinho so olha a distancia ate o
        gol: com a linha de tiro bloqueada por um adversario, a jogada caia no
        caso 6 (manter a direcao do gol) e mesmo assim armava o chute - ou seja,
        chutava contra as costas do adversario parado na frente.
        """
        alvo_x, alvo_y, tipo = self._alvo_da_jogada()
        if tipo == "passe":
            return True
        return (self._can_kick()
                and self._linha_livre(self.ball.position_x, self.ball.position_y,
                                      alvo_x, alvo_y, FOLGA_LINHA_TIRO))

    def _versor_bola_gol_puro(self):
        """Versor da bola para o GOL adversario, sem considerar passe.

        POR QUE EXISTE, separado de _versor_bola_gol: o posicionamento dos
        apoiadores nao pode depender do alvo da jogada, porque o alvo da jogada
        PODE SER UM APOIADOR. Isso fecha um laco - a posicao de apoio passa a se
        definir em funcao de si mesma e os robos saem fugindo. Medimos: os
        apoiadores se afastaram de 1442 e 2419 mm para 3252 e 3439 mm da bola,
        saindo do alcance de passe (DIST_MAX_PASSE) e derrubando a jogada
        inteira - o chute nao armava em nenhuma execucao.

        O apoio se posiciona sempre em relacao ao GOL, que e uma referencia
        fixa. Quem decide para onde a bola vai e _alvo_da_jogada.
        """
        dx = self.attack_goal.x - self.ball.position_x
        dy = self.attack_goal.y - self.ball.position_y
        n = hypot(dx, dy) or 1.0
        return dx / n, dy / n

    def _versor_bola_gol(self):
        mx, my, _ = self._alvo_da_jogada()
        dx = mx - self.ball.position_x
        dy = my - self.ball.position_y
        n = hypot(dx, dy) or 1.0
        return dx / n, dy / n

    def _posicao_de_apoio(self, ordem):
        """Linha de passe: a frente da bola, aberto para um dos lados.

        Usa o versor PURO (bola->gol) de proposito - ver
        _versor_bola_gol_puro para o laco que isso evita.
        """
        ux, uy = self._versor_bola_gol_puro()
        px, py = -uy, ux                       # perpendicular a linha bola->gol
        lado = 1.0 if ordem % 2 == 0 else -1.0
        avanco = 1200.0 + 300.0 * (ordem // 2)
        abertura = 1100.0 + 400.0 * (ordem // 2)
        x = self.ball.position_x + ux * avanco + px * abertura * lado
        y = self.ball.position_y + uy * avanco + py * abertura * lado
        return self._dentro_do_campo(x, y)

    # ------------------------------------------------------------- acoes
    def _passo_ate(self, robot_id, alvo_x, alvo_y):
        """Reduz um alvo distante a um passo curto a partir da posicao MEDIDA.

        Ver PASSO_MAX para a medicao que justifica isto. Em resumo: o driver
        executa a trajetoria pelo relogio, sem realimentacao, e o controlador nao
        tem frenagem - entao pedir um alvo longe faz o robo ou nao sair do lugar
        ou disparar e passar mais de um metro alem. Pedindo pouco de cada vez,
        ele obedece.
        """
        p = self._pos(robot_id)
        if p is None:
            return alvo_x, alvo_y
        dx, dy = alvo_x - p[0], alvo_y - p[1]
        d = hypot(dx, dy)

        # Duas regras que NAO podem se anular:
        #
        #   - frenagem: o passo e uma fracao do que falta, para o robo
        #     desacelerar na chegada (FRACAO_FREIO);
        #   - desempacamento: se ele nao esta saindo do lugar, o passo cresce
        #     ate vencer a zona morta do controlador (PASSO_EMPACADO).
        #
        # ERRO QUE ISTO CORRIGE: antes era
        #     max(PASSO_MIN, min(passo_atual, d * FRACAO_FREIO))
        # e o 'min' fazia a frenagem ENGOLIR o desempacamento - com 350 mm para
        # andar o passo nunca passava de 175 mm, por mais empacado que o robo
        # estivesse. Medimos o resultado: 105 mm percorridos em 25 segundos, com
        # a rampa anti-empacamento ativa e inutil.
        #
        # Agora o desempacamento entra por FORA da frenagem: freia normalmente,
        # mas se o robo nao responde, empurra mais forte de qualquer jeito.
        base = max(PASSO_MIN, min(PASSO_MAX, d * FRACAO_FREIO))
        desempacar = self._passo_de_desempacamento(robot_id)
        passo = max(base, desempacar)
        if d <= passo:
            # ALVO PERTO E ROBO EMPACADO: estende o alvo ALEM dele.
            #
            # A rampa anti-empacamento era INERTE justamente no caso para o qual
            # foi criada. Ela cresce o PASSO, mas quando o alvo ja esta mais
            # perto que o passo esta linha devolvia o alvo intacto - e o erro
            # comandado continuava pequeno. Com kp=1,5, um erro de 100 mm pede
            # 0,15 m/s, abaixo dos ~0,35 m/s que o robo precisa para sair do
            # lugar (§16.3): ele fica parado para sempre, e a rampa cresce sem
            # nunca mudar o que e comandado.
            #
            # MEDIDO: em 4 de 12 execucoes o cobrador morria no ponto de encaixe.
            # A assinatura e inconfundivel - o ponto fica 250 mm atras da bola e
            # a placa a 75,5 mm do centro, entao parar ali da xx ~ 174; medimos
            # xx = 184, 145, 121 e 114, com a bola andando 0 a 289 mm. Nessas
            # execucoes a janela do chutador NUNCA abriu.
            #
            # Estendendo o alvo, a DIRECAO nao muda - so a amplitude do erro,
            # ate vencer a zona morta. E transitorio por construcao: assim que o
            # robo anda, _passo_de_desempacamento zera e a frenagem normal volta
            # no ciclo seguinte.
            # EXTENSAO DO ALVO: implementada, medida, e deixada DESLIGADA.
            #
            # O defeito que ela corrige e real e esta provado: a rampa
            # anti-empacamento cresce o PASSO, mas com o alvo mais perto que o
            # passo esta linha devolve o alvo intacto, entao o que e COMANDADO
            # nunca muda. Robo parado a 100 mm do alvo pede 0,15 m/s com kp=1,5,
            # abaixo dos ~0,35 m/s da zona morta (§16.3) - e continua pedindo
            # isso para sempre, por mais que a rampa suba. A rampa e inerte
            # exatamente no caso para o qual foi criada.
            #
            # A correcao (estender o alvo ate a distancia da rampa, na MESMA
            # direcao) foi testada: os gols foram de 4/12 para 5/12 e o balde
            # "nao chegou" de 4 para 3 - dentro do ruido. E o A/B na oscilacao
            # foi inconclusivo (10/15/11 inversoes com, 13/3/15 sem).
            #
            # Fica desligada por duas razoes. Primeiro, o §14: o que nao melhora
            # de forma separavel do ruido nao entra. Segundo, e um CONTORNO de
            # um defeito que mora fora daqui - medimos o robo a 142 mm do
            # setpoint do driver na mediana e 36% do tempo acima de 200 mm (ver
            # HANDOVER §34). Pedir um alvo mais longe para vencer a zona morta
            # e remendo sobre remendo enquanto a malha estiver aberta.
            #
            # RELIGADA em 25/08, agora que o yy deixou de mascarar o problema.
            #
            # Com o feedforward removido (ajuste 'controle'), o yy caiu de 82
            # para 28,5 mm de mediana e passou a ficar dentro do limite em 7 de
            # 12 execucoes. Isso deixou o xx sozinho como gargalo: mediana de
            # 86 mm contra os 31,5 que o grSim exige, com 1 de 12 dentro.
            #
            # xx = 86 quer dizer que a bola esta a 161 mm do centro do robo,
            # quando o contato fisico acontece a 111: ele para 50 mm ANTES de
            # encostar. E 50 mm com kp=1,5 pedem 0,075 m/s, muito abaixo dos
            # ~0,35 m/s que o robo precisa para sair do lugar (§16.3). Ele nao
            # consegue andar esse ultimo trecho - que e exatamente o caso para o
            # qual esta extensao existe.
            if desempacar > d > 1.0:
                return p[0] + dx * desempacar / d, p[1] + dy * desempacar / d
            return alvo_x, alvo_y
        return p[0] + dx * passo / d, p[1] + dy * passo / d

    def _passo_de_desempacamento(self, robot_id) -> float:
        """Passo extra enquanto o robo estiver empacado; 0 se ele esta andando.

        Ver PASSO_EMPACADO: com passo curto o comando de velocidade fica baixo e
        em varias execucoes o robo simplesmente nao sai do lugar.
        """
        r = self.ally_robots.get(robot_id)
        if r is None:
            return 0.0
        parado = hypot(r.velocity_x, r.velocity_y) < VEL_EMPACADO
        n = self.estado.empacado.get(robot_id, 0) + 1 if parado else 0
        self.estado.empacado[robot_id] = n
        if n <= CICLOS_EMPACADO:
            return 0.0
        return min(PASSO_EMPACADO, PASSO_MAX + 30.0 * (n - CICLOS_EMPACADO))

    def _aproximar_da_bola(self, robot_id):
        """P4: vai para um ponto ATRAS da bola, nao para cima dela.

        Antes o alvo era a propria posicao da bola e ao mesmo tempo o comando
        marcava ball=True, o que faz o obstacle_factory criar um circulo de 60 mm
        sobre a bola. O alvo caia dentro do obstaculo: o planejador parava antes,
        o robo nunca fechava os 300 mm exigidos por _robot_close_to_ball,
        _go_to_goal nunca rodava e o chute nunca era sequer avaliado.
        """
        ux, uy = self._versor_bola_gol()
        alvo_x = self.ball.position_x - ux * DIST_ATRAS_DA_BOLA
        alvo_y = self.ball.position_y - uy * DIST_ATRAS_DA_BOLA
        alvo_x, alvo_y = self._dentro_do_campo(alvo_x, alvo_y)
        alvo_x, alvo_y = self._passo_ate(robot_id, alvo_x, alvo_y)

        comando = self.skills_factory.move_with_angle(
            robot_id=robot_id, target_x=alvo_x, target_y=alvo_y,
            vel_x=0.0, vel_y=0.0, angle=atan2(uy, ux),
        )
        # bola como obstaculo enquanto contorna; o alvo agora esta FORA dela
        self._obstaculos(comando, robot_id, evitar_bola=True)
        return self._armar_se_der(comando, robot_id)

    def _empurrar_para_o_gol(self, robot_id):
        """Ja esta atras da bola: empurra na linha do gol e chuta se puder."""
        # Alvo = um pouco ALEM da bola, para o robo continuar pressionando.
        #
        # Historico: primeiro o alvo era +220 mm e o robo passava batido, de
        # raspao. Mirando a PROPRIA bola ele parava colado - mas parava a 122 mm,
        # e o chutador do grSim exige menos que isso.
        #
        # A conta (robot.cpp:128): a bola tem de estar a menos de
        # KickerThickness*2 + BallRadius = 5*2 + 21,5 = 31,5 mm da PLACA do
        # chutador, que fica a ~90 mm do centro. Ou seja, ~121,5 mm entre
        # centros - e mediamos exatamente 122 mm, no limite, o que explica o
        # chute sair uma vez e falhar na seguinte.
        #
        # Com o alvo 150 mm alem da bola o robo segue empurrando e o vao fica
        # abaixo do limite; o desvio lateral (max 80 mm) e a mira apertada
        # (0,15 rad) impedem que ele passe de raspao como antes.
        ux, uy = self._versor_bola_gol()

        # PASSO SOBRE A LINHA, nao em direcao a bola.
        #
        # Andar direto para a bola a partir de um ponto fora da linha e uma
        # diagonal: ela PRESERVA o erro lateral ate o momento do contato, e o
        # robo ombreia a bola em vez de empurra-la. Medido em duas execucoes
        # perdidas: com 86 mm de desvio lateral o robo bateu de lado e a bola
        # saiu para (2609,216), subindo em vez de ir ao gol; a execucao que fez
        # GOL foi justamente a unica em que ele chegou parado e sobre a linha,
        # com a bola saindo de (2538,-3) e desviando so 36 mm em 500 mm.
        #
        # Colocando o alvo SOBRE a linha bola->gol, o erro lateral e corrigido
        # durante a aproximacao e o contato final acontece no eixo do corpo -
        # que e exatamente o que o chutador do grSim exige (robot.cpp:128, a
        # bola precisa estar a menos de KickerWidth/2 = 40 mm do eixo).
        s_atual = self._projecao_na_linha(robot_id)          # negativo = atras

        # O alvo passa ALEM da bola, de proposito.
        #
        # Parando o alvo na bola, o erro de posicao vai a zero junto com o
        # contato e o robo estaciona no primeiro toque - medimos ele parando a
        # 107-116 mm do centro da bola. Mas o chutador do grSim so dispara com a
        # bola a menos de KickerThickness*2 + BallRadius = 31,5 mm da PLACA
        # (robot.cpp:128), que fica a CenterFromKicker = 73 mm do centro: ou
        # seja, no maximo ~104 mm entre centros.
        #
        # Faltavam cerca de 10 mm. Consequencia medida: a bola saia a
        # 1,2-3,4 m/s (empurrao do corpo) em vez dos 6,0 m/s do chute, e morria
        # 300 mm antes da linha de fundo - com a direcao ja perfeita
        # (y final entre -57 e -9).
        #
        # Com o alvo alem da bola o robo continua pressionando, o vao fecha
        # abaixo dos 104 mm e a placa encosta. O avanco e limitado a
        # ALEM_DA_BOLA para ele nao sair correndo atras da bola depois do chute;
        # de qualquer forma _recuar_apos_chute assume assim que o chute e
        # registrado.
        # RETO, DE UMA VEZ, ALEM DA BOLA.
        #
        # Antes o alvo avancava em passos (s_atual + PASSO_MAX) e parava NA bola
        # (ALEM_DA_BOLA = 0). Dois problemas, ambos medidos:
        #   - parando na bola, o erro de posicao vai a zero junto com o contato
        #     e o robo estaciona a ~118 mm do centro dela, enquanto a placa do
        #     chutador exige no maximo ~107 mm: o chute nunca dispara;
        #   - em passos curtos, o comando vira kp vezes um erro pequeno, abaixo
        #     dos ~0,35 m/s que o robo precisa para sair do lugar.
        #
        # Com um alvo fixo alem da bola o erro permanece grande, a velocidade
        # permanece util, e o robo ATRAVESSA o ponto de contato em vez de parar
        # nele. E o que uma cobranca e: apontar para o canto e ir reto.
        # PARA DE AVANCAR QUANDO A PLACA JA ALCANCA A BOLA.
        #
        # O alvo alem da bola serve para o robo ATRAVESSAR o ponto de contato em
        # vez de estacionar antes dele. Mas depois que a placa chega na bola,
        # continuar avancando so faz mal: o grSim subtrai velocidade da bola a
        # cada contato (KickerDampFactor, robot.cpp:168), e o robo alcanca a bola
        # que acabou de sair.
        #
        # MEDIDO: com o chute disparando e a bola bem centrada (yy de 20 a 25 mm),
        # ela percorria 1306, 1482 e 1679 mm - quando 6,4 m/s no atrito medido
        # rendem 2362 mm. Faltavam ~40% da energia, roubados pelo proprio robo.
        #
        # Dentro do alcance da placa o alvo passa a ser a posicao ATUAL: o
        # chutador dispara no contato de qualquer forma, e o robo para de
        # empurrar. E o gesto de uma cobranca de verdade - bate e nao segue.
        # REMOVIDA a frenagem por xx (era XX_PARA_SEGURAR = 60 mm).
        #
        # Ela mandava o robo parar - alvo = posicao atual - assim que a placa
        # chegava a menos de 60 mm da bola. Mas o grSim so dispara com a bola a
        # menos de 31,5 mm da placa (robot.cpp:128). O robo freava DENTRO da
        # faixa 31,5-60, onde chutar e impossivel: a margem estava aplicada ao
        # contrario, dando folga para parar antes em vez de folga para chegar.
        #
        # Medido em 12 execucoes com ela ativa: xx ficou em 14, 24, 50, 57, 65,
        # 70, 87, 99, 115, 127, 140, 151, 160 - UMA entrou na janela de 31,5.
        #
        # Ela foi criada para impedir que o robo roubasse energia da bola ao
        # continuar avancando. Essa premissa nao se sustenta: lendo a visao crua
        # do grSim, a bola sai a 6103-6207 mm/s dos 6400 comandados - 96% da
        # energia chega. O HANDOVER §24 ja registrava a hipotese como "testada e
        # nao confirmada", e o §27 registra que esta iteracao deu 1 gol em 6
        # contra os 2 em 6 da melhor configuracao. Nunca foi revertida.
        #
        # O motivo legitimo dela - nao empurrar a bola depois do chute - continua
        # coberto por _recuar_apos_chute, que para o robo no lugar assim que o
        # chute e registrado. La a frenagem acontece DEPOIS do disparo, que e
        # quando ela faz sentido.
        alvo_x = self.ball.position_x + ux * AVANCO_RETO
        alvo_y = self.ball.position_y + uy * AVANCO_RETO

        # TENTADA E REVERTIDA: correcao lateral durante o empurrao.
        #
        # A geometria estava certa - deslocar o alvo de o = L*(1 - 1/f), com
        # f = -s/(AVANCO_RETO - s), faz a reta cruzar a linha exatamente na
        # bola, e num teste deterministico levava o desvio de 23-105 mm para 0.
        #
        # No simulador NAO se confirmou: 12 execucoes, 0 gols e 0 disparos,
        # contra 1 e 1 sem ela. O defeito e que a correcao DIVERGE na reta
        # final: quando s -> 0 (o robo chegando na bola) f -> 0 e o alvo e
        # jogado para o lado ate saturar no teto, entao o robo passa a andar de
        # banda em vez de fechar. Media o efeito: xx otimo (3,2 / 8,6 / 28,5 mm)
        # e yy grande do mesmo jeito (66, 88, 130 mm).
        #
        # v2 TAMBEM REVERTIDA. Congelamos a correcao nos ultimos 150 mm para
        # matar a manobra lateral que afundou a v1, e a convergencia continua
        # boa no papel: simulando ciclo a ciclo, o desvio no ponto da bola caia
        # de 23-88 mm para 11-50 mm, dentro dos 40 para desvio inicial ate
        # 120 mm.
        #
        # No simulador: 1 gol em 6, contra 5 em 12 sem ela. E pior, a janela do
        # chutador chegou a ficar aberta 32 e 76 quadros SEM disparo - mais
        # oportunidade perdida do que antes, nao menos.
        #
        # A leitura honesta e que curvar a trajetoria de aproximacao atrapalha
        # algo que ainda nao medimos - provavelmente a orientacao, que e
        # comandada como angulo fixo enquanto o caminho deixa de ser reto.
        # Duas tentativas, duas reversoes: o proximo a mexer aqui deve MEDIR
        # primeiro o desvio lateral ao longo do tempo, e nao propor outra
        # formula no escuro.
        alvo_x, alvo_y = self._dentro_do_campo(alvo_x, alvo_y)

        # Aponta para a BOLA, nao para o gol.
        #
        # grSim robot.cpp:128 exige que a bola esteja a menos de
        # KickerWidth/2 = 40 mm do EIXO DO CORPO do robo. Apontando para o gol,
        # qualquer desalinhamento joga a bola para fora da placa: medimos o robo
        # a 104 mm da bola (dentro do alcance) com o chute comandado a 5.8 e a
        # bola atingindo so 1458 mm/s - ou seja, empurrao, o chutador nao
        # disparou.
        #
        # Apontando para a bola, o eixo do corpo passa por ela e o teste lateral
        # fecha. E o tiro sai nessa mesma direcao (robot.cpp:157 usa
        # getBodyDirection) - que aponta ao gol de qualquer forma, porque o robo
        # so chega aqui vindo de TRAS da bola, na linha bola->gol, e a trava
        # _mira_esta_boa exige o corpo a 0,15 rad da direcao do gol.
        # O CORPO APONTA NA DIRECAO DA MIRA, nao para a bola.
        #
        # O tiro sai no eixo do corpo (grSim robot.cpp:157, getBodyDirection).
        # Apontando para a bola, o angulo pedido MUDA a cada ciclo conforme o
        # robo anda - e o controlador de orientacao e so proporcional (kp=1),
        # entao ele persegue um alvo movel e chega sempre atrasado. Medimos o
        # efeito: mirando o canto em y=-350, a bola saiu para y=-581, cerca de
        # 10 graus mais aberta do que o pedido.
        #
        # Com a direcao da mira o angulo fica CONSTANTE durante todo o empurrao
        # e o controlador tem tempo de assentar. So e seguro porque a
        # aproximacao agora e axial: medimos o desalinhamento lateral no
        # chutador (o yy do robot.cpp:128) em 3,0 mm, contra um limite de
        # 40 mm - apontar na linha nao ameaca o contato com a placa.
        ang = atan2(uy, ux)

        comando = self.skills_factory.move_with_angle(
            robot_id=robot_id, target_x=alvo_x, target_y=alvo_y,
            vel_x=0.0, vel_y=0.0, angle=ang,
        )
        # aqui a bola NAO e obstaculo: a intencao e justamente toca-la
        self._obstaculos(comando, robot_id, evitar_bola=False)

        # O chute so e liberado com a bola EFETIVAMENTE ao alcance do chutador
        # E com o robo posicionado ATRAS dela, na linha do gol.
        # O chute fica ARMADO durante toda a aproximacao final, em vez de ser
        # decidido no ultimo instante.
        #
        # Armar cedo nao tem custo: o grSim so dispara quando a bola encosta na
        # placa (robot.cpp:160). Ja decidir no ultimo instante custava a jogada
        # inteira - a janela de contato dura uma amostra, e as travas piscando
        # deixavam o chute em zero justamente nela. Dai o sintoma de sempre
        # "furar ou empurrar" a bola em vez de chuta-la.
        #
        # O que continua sendo exigido: haver alvo valido (gol livre ou
        # companheiro), nao violar duplo toque, e o corpo estar apontado para a
        # bola dentro da largura da placa.
        return self._armar_se_der(comando, robot_id)

    def _recuar_apos_chute(self, robot_id):
        """Depois de chutar, o cobrador recua e libera a bola."""
        # PARAR NO LUGAR, nao recuar.
        #
        # Recuar para 600 mm atras da bola exige frear e inverter o sentido, e
        # nesse meio tempo o robo ainda avanca - alcancando a bola recem-chutada e
        # matando a energia dela. Medimos TRES picos de velocidade na mesma
        # cobranca (8367, 7136 e 5169 mm/s): chute, re-contato, re-contato.
        #
        # Parar onde esta e a frenagem mais curta possivel. A bola sai a 6 m/s e
        # o robo faz no maximo ~2 m/s, entao ela escapa sozinha.
        #
        # (Alvo fixo, nunca relativo a posicao atual: calculando a partir do robo
        # o alvo fugia junto com ele - medimos 4273 mm de fuga infinita.)
        p_atual = self._pos(robot_id)
        ux, uy = self._versor_bola_gol()
        if p_atual is None:
            alvo_x = self.ball.position_x - ux * 600.0
            alvo_y = self.ball.position_y - uy * 600.0
        else:
            alvo_x, alvo_y = p_atual
        alvo_x, alvo_y = self._dentro_do_campo(alvo_x, alvo_y)
        comando = self.skills_factory.move_with_angle(
            robot_id=robot_id, target_x=alvo_x, target_y=alvo_y,
            vel_x=0.0, vel_y=0.0, angle=atan2(uy, ux),
        )
        self._obstaculos(comando, robot_id, evitar_bola=True)
        comando.deactivate_kick()
        return comando

    def _robot_close_to_ball(self, robot_id) -> bool:
        return self._dist_ate_bola(robot_id) < DIST_TOQUE

    def _pronto_para_empurrar(self, robot_id) -> bool:
        """Ja pode partir para o empurrao e o chute?

        Duas portas de entrada, para nao depender de uma so:
          - chegou ao ponto de aproximacao (atras da bola, na linha do gol), ou
          - ja esta dentro da faixa de toque da bola.

        Antes so existia a segunda, e o ponto de aproximacao ficava a 340 mm da
        bola enquanto a faixa de toque era 300 mm: o cobrador estacionava no
        ponto e a jogada nunca avancava para o chute.
        """
        p = self._pos(robot_id)
        if p is None:
            return False

        # Desalinhado em relacao a linha bola->gol? Entao ainda nao: empurrar
        # torto faz o robo passar de raspao pela bola em vez de toca-la.
        if self._desvio_lateral(robot_id) > DESVIO_LATERAL_MAX:
            return False

        ux, uy = self._versor_bola_gol()
        px = self.ball.position_x - ux * DIST_ATRAS_DA_BOLA
        py = self.ball.position_y - uy * DIST_ATRAS_DA_BOLA
        no_ponto = hypot(p[0] - px, p[1] - py) < DIST_NO_PONTO

        if no_ponto:
            # PARAR antes de avancar: so libera o empurrao com o robo ja
            # praticamente imovel no ponto de aproximacao.
            #
            # Sem isso ele chegava com velocidade e ATROPELAVA a bola de lado -
            # medimos colisoes com o corpo a 98 graus da bola, mandando-a para o
            # nosso proprio campo. Uma cobranca de falta e deliberada: o robo se
            # posiciona, para, alinha, e so entao avanca em linha reta.
            return self._robot_is_stable(robot_id)

        # ja encostado na bola (por exemplo, ela rolou ate ele): segue em frente
        return self._robot_close_to_ball(robot_id)

    def _projecao_na_linha(self, robot_id) -> float:
        """Quanto o robo esta a FRENTE da bola, na direcao do gol.

        Positivo = do lado do gol (atrapalhando). Negativo = atras da bola, que e
        de onde a cobranca deve sair.
        """
        p = self._pos(robot_id)
        if p is None:
            return 0.0
        ux, uy = self._versor_bola_gol()
        return (p[0] - self.ball.position_x) * ux + (p[1] - self.ball.position_y) * uy

    def _fase_da_cobranca(self, robot_id) -> str:
        """Em que fase da cobranca este robo esta.

        'contornar' -> esta a frente da bola ou muito fora da linha: precisa dar
                       a volta POR FORA, sem cruzar a frente da bola;
        'encaixar'  -> ja esta atras, mas ainda nao no ponto de aproximacao;
        'empurrar'  -> encaixado e alinhado: avanca e chuta.

        Sem estas fases o robo usava a rota mais curta ate o ponto de
        aproximacao, o que frequentemente passa NA FRENTE da bola - e ele acabava
        entre a bola e o gol, chutando para o lado errado.
        """
        projecao = self._projecao_na_linha(robot_id)

        # A fase 'empurrar' e TRAVADA: uma vez iniciado, o empurrao vai ate o
        # fim. Ver EstadoFreekick.fase_travada para a medicao que motivou isto -
        # sem a trava a fase oscilava a cada ciclo e o alvo comandado pulava
        # entre a bola e o ponto de encaixe 30 vezes por segundo.
        #
        # A trava so cai por perda REAL de geometria, nao por o robo estar em
        # movimento (que e justamente o que se espera dele empurrando):
        #   - passou da bola (projecao >= -MARGEM_ATRAS), ou
        #   - saiu da linha bola->gol alem de DESVIO_TRAVADO.
        # O chute em si continua com as travas de sempre (_atras_da_bola,
        # _mira_esta_boa): travar a fase nao afrouxa nada do disparo.
        if self.estado.fase_travada.get(robot_id) == "empurrar":
            if (projecao < -MARGEM_ATRAS
                    and self._desvio_lateral(robot_id) <= DESVIO_TRAVADO):
                return "empurrar"
            self.estado.fase_travada.pop(robot_id, None)

        if projecao > -MARGEM_ATRAS:
            self.estado.fase_travada.pop(robot_id, None)
            return "contornar"
        if self._pronto_para_empurrar(robot_id):
            self.estado.fase_travada[robot_id] = "empurrar"
            return "empurrar"
        return "encaixar"

    def _ponto_de_contorno(self, robot_id):
        """Proximo passo de uma ORBITA em torno da bola.

        Nao devolve um ponto fixo atras da bola: devolve o proximo ponto de um
        arco, mantendo o robo a um raio seguro enquanto ele gira ate ficar atras
        dela.

        POR QUE ORBITAR, e nao ir direto:
        o alvo "atras da bola" e um ponto so, e o planejador traca a rota mais
        curta ate ele - que atravessa a bola. Pior: a bola entra como obstaculo
        de apenas 60 mm (obstacle_factory.py:44), menor que o raio do robo
        (90 mm), entao o planejador desvia o CENTRO e o corpo passa por cima
        mesmo assim. Medimos a colisao: pico de 45868 mm/s aos 7 s, com o corpo a
        98 graus da bola - a bola saiu empurrada, nao chutada.

        Girando em passos de ~50 graus por vez, cada alvo intermediario fica
        longe da bola e a rota curta entre eles nunca cruza por cima dela.
        """
        p = self._pos(robot_id)
        bx, by = self.ball.position_x, self.ball.position_y
        ux, uy = self._versor_bola_gol()

        # PASSOU DA BOLA DE RASPAO? Volte para o ponto de cobranca, nao orbite.
        #
        # A orbita existe para quando o robo esta LONGE e do lado errado. Quando
        # ele apenas ultrapassou a bola sem toca-la - que e o que acontece
        # quando erra o alinhamento - a orbita o manda para 300-700 mm de
        # distancia e a jogada se desfaz. Medimos exatamente isso: projecao
        # indo de -238 para +369 em 1,2 s, fase virando 'contornar', e o robo
        # terminando a 2820 mm da bola sem nunca mais voltar.
        #
        # Estando perto, o caminho de volta e simplesmente RECUAR pela propria
        # linha ate o ponto de cobranca. E o que um jogador faz: errou a bola,
        # volta e tenta de novo.
        if p is not None and hypot(p[0] - bx, p[1] - by) < 500.0:
            return self._dentro_do_campo(bx - ux * DIST_ATRAS_DA_BOLA,
                                         by - uy * DIST_ATRAS_DA_BOLA)

        if p is None:
            return self._dentro_do_campo(bx - ux * RAIO_CONTORNO,
                                         by - uy * RAIO_CONTORNO)

        # angulo atual do robo em torno da bola, e o angulo de destino (atras)
        ang_atual = atan2(p[1] - by, p[0] - bx)
        ang_destino = atan2(-uy, -ux)

        # menor diferenca angular, limitada a um passo por ciclo
        delta = (ang_destino - ang_atual + pi) % (2 * pi) - pi
        passo = max(-0.9, min(0.9, delta))          # ~50 graus
        ang_novo = ang_atual + passo

        # FECHA o raio a cada ciclo, ate RAIO_CONTORNO. Antes era
        #     raio = max(RAIO_CONTORNO, dist_atual)
        # que nunca diminui: qualquer afastamento virava o raio novo, e o raio
        # novo garantia o afastamento no ciclo seguinte. Uma catraca - o robo
        # saia e NAO VOLTAVA MAIS.
        #
        # MEDIDO numa execucao (distancia robo-bola ao longo do tempo):
        #     172 mm -> 1003 -> 164 -> 2449 -> 3000 -> 989 -> 1437
        # com 12 inversoes de sentido. E o "vai pra frente e pra tras" que o
        # Felipe viu nas 3 execucoes dele, e a razao de a bola nunca ser tocada.
        #
        # A intencao original - nao raspar na bola durante a volta - continua
        # atendida: o raio so encolhe PASSO_MAX por ciclo e nunca fica abaixo de
        # RAIO_CONTORNO, entao o caminho jamais corta por cima da bola. A
        # diferenca e que agora ele converge para a orbita em vez de fugir dela.
        raio = max(RAIO_CONTORNO, hypot(p[0] - bx, p[1] - by) - PASSO_MAX)
        return self._dentro_do_campo(bx + raio * cos(ang_novo),
                                     by + raio * sin(ang_novo))

    def _contornar_a_bola(self, robot_id):
        """Fase 1: dar a volta por fora, com a bola como obstaculo."""
        alvo_x, alvo_y = self._ponto_de_contorno(robot_id)
        alvo_x, alvo_y = self._passo_ate(robot_id, alvo_x, alvo_y)

        # ANGULO CONSTANTE EM TODAS AS FASES - a direcao da mira.
        #
        # Antes esta fase comandava "aponte para a bola a partir do proximo
        # ponto da orbita". Como esse ponto se move a cada ciclo, o alvo angular
        # tambem se movia - e entre fases ele saltava de vez.
        #
        # MEDIDO: a velocidade angular comandada saturava alternando entre +2,0
        # e -2,0 rad/s (os extremos do controlador) e o robo girava sem parar.
        # Girando, a cinematica inversa converte a velocidade no referencial
        # errado - o robo passou a acelerar para LONGE da bola com a saida
        # saturada em (-1,5 , +1,5) m/s, chegando a 3,2 m de distancia.
        # Realimentacao positiva: gira, erra a direcao, se afasta, gira mais.
        #
        # O robo e holonomico: ele contorna de lado sem precisar virar. Manter o
        # angulo fixo na direcao do chute elimina o giro, e com ele o
        # descontrole - e ainda deixa o corpo ja apontado quando chegar a hora.
        ux_mira, uy_mira = self._versor_bola_gol()
        comando = self.skills_factory.move_with_angle(
            robot_id=robot_id, target_x=alvo_x, target_y=alvo_y,
            vel_x=0.0, vel_y=0.0,
            angle=atan2(uy_mira, ux_mira),
        )
        self._obstaculos(comando, robot_id, evitar_bola=True)
        return self._armar_se_der(comando, robot_id)

    def _atras_da_bola(self, robot_id) -> bool:
        """O robo esta ATRAS da bola, na linha que leva ao gol?

        Compara a direcao robo->bola com a direcao bola->gol. Se as duas
        coincidem, empurrar a bola a manda para o gol.

        POR QUE ISSO FALTAVA: passamos a apontar o corpo para a BOLA (necessario
        para o teste lateral do chutador, robot.cpp:128). Isso resolveu o
        disparo - medimos a bola saindo a 11 m/s - mas o tiro sai no EIXO DO
        CORPO. Encostando de lado, o robo chutava a bola para tras: numa
        execucao ela foi de (2500,0) para (2299,285), na direcao do NOSSO campo.

        Com esta trava, apontar para a bola e apontar para o gol.
        """
        p = self._pos(robot_id)
        if p is None:
            return False
        ux, uy = self._versor_bola_gol()
        dx = self.ball.position_x - p[0]
        dy = self.ball.position_y - p[1]
        n = hypot(dx, dy) or 1.0
        # cosseno entre robo->bola e bola->gol
        # 0.985 = ~10 graus. Com 20 graus (0.94) a bola saia desviada: medimos
        # ela indo de (2500,0) para (3401,-250), ou seja 15 graus fora - o que a
        # 2 m do gol da 555 mm de desvio, mais que a meia-largura de 500 mm.
        return (dx / n) * ux + (dy / n) * uy >= 0.985

    def _desvio_lateral(self, robot_id) -> float:
        """Distancia do robo ate a reta que liga a bola ao gol adversario.

        E a medida que faltava: o robo podia estar "perto da bola" e mesmo assim
        completamente fora da linha de empurrao, passando ao lado dela.
        """
        p = self._pos(robot_id)
        if p is None:
            return float("inf")
        ux, uy = self._versor_bola_gol()
        dx = p[0] - self.ball.position_x
        dy = p[1] - self.ball.position_y
        return abs(dx * (-uy) + dy * ux)      # componente perpendicular

    # ------------------------------------------------------------ execucao
    def execute(self):
        """P2: devolve (TaskStatus, lista).

        Antes devolvia so a lista, mas plays/freekick.py fazia
        'status, commands = executor.execute()'. Com 3 robos isso levantava
        ValueError a cada ciclo; com exatamente 2 nao levantava nada e
        silenciosamente jogava fora o comando do primeiro robo.
        """
        # SUCCESS, e nao FAILURE, de proposito.
        #
        # O Selector de plays/freekick.py cai para TheirFreekickAction assim que
        # OurFreekick falha. Devolvendo FAILURE aqui, um timeout na NOSSA
        # cobranca fazia o time passar a se posicionar como se a falta fosse do
        # adversario - o cobrador largava a bola e recuava para o ponto de
        # bloqueio. Com SUCCESS a jogada segue sendo nossa, apenas em modo de
        # parada segura.
        if self._check_timeout():
            return TaskStatus.SUCCESS, self._parada_segura("tempo esgotado")
        if not self._is_ball_position_valid():
            return TaskStatus.SUCCESS, self._parada_segura("bola em posicao invalida")
        if not self.ally_robots:
            return TaskStatus.RUNNING, []

        comandos = []

        if 0 in self.ally_robots:
            comandos.append(
                GoalkeeperKickoff().execute(
                    self.gk_target, self.gk_angle,
                    ally_ids=list(self.ally_robots),
                    enemy_ids=list(self.enemy_robots),
                )
            )

        cobrador = self._eleger_cobrador()
        ordem_apoio = 0
        for rid in self._ids_de_linha():
            if rid == cobrador:
                if self.estado.last_kicker_id == rid:
                    # JA CHUTOU: para de avancar IMEDIATAMENTE.
                    #
                    # O registro so acontece com a bola ja a mais de 2000 mm/s
                    # (chute real), entao recuar na hora e seguro - e necessario:
                    # continuando a avancar, o robo alcanca a bola e a FREIA
                    # (robot.cpp:168 subtrai velocidade no contato). Medimos um
                    # chute de 6 m/s - que no teste de atrito percorre 4220 mm -
                    # render apenas 385 mm por causa disso.
                    #
                    # Antes eu esperava a bola estar a 400 mm para recuar, o que
                    # era tarde demais: a frenagem ja tinha acontecido.
                    #
                    # A condicao de distancia e essencial: 'last_kicker_id' e
                    # marcado quando o COMANDO de chute sai, nao quando a bola
                    # parte. Sem ela o robo recuava antes mesmo de encostar -
                    # medimos ele fugindo 4273 mm com a bola parada no lugar.
                    #
                    # grSim robot.cpp:168 subtrai velocidade da bola no contato
                    # (KickerDampFactor). Continuando a empurrar depois do
                    # chute, o robo alcanca a bola e a FREIA: medimos o chute
                    # saindo a 6 m/s - que no teste de atrito percorre 4220 mm -
                    # e a bola parando com 350 mm.
                    #
                    # Tambem protege a regra 8.2 (duplo toque).
                    comandos.append(self._recuar_apos_chute(rid))
                else:
                    fase = self._fase_da_cobranca(rid)
                    if fase == "empurrar":
                        comandos.append(self._empurrar_para_o_gol(rid))
                    elif fase == "encaixar":
                        comandos.append(self._aproximar_da_bola(rid))
                    else:
                        comandos.append(self._contornar_a_bola(rid))
            else:
                alvo_x, alvo_y = self._posicao_de_apoio(ordem_apoio)
                ordem_apoio += 1
                cmd = self.skills_factory.move_with_angle(
                    robot_id=rid, target_x=alvo_x, target_y=alvo_y,
                    vel_x=0.0, vel_y=0.0,
                    angle=atan2(self.ball.position_y - alvo_y,
                                self.ball.position_x - alvo_x),
                )
                self._obstaculos(cmd, rid, evitar_bola=True)
                cmd.deactivate_kick()
                comandos.append(cmd)

        # Registra o chute apenas quando a BOLA REALMENTE PARTIU.
        #
        # Antes bastava o comando sair com kick > 0. So que comandar nao e
        # chutar: o grSim so dispara em contato (robot.cpp:160). Se o chute nao
        # pegava, o robo ficava marcado como "ja chutou" para sempre - parava de
        # avancar e recuava indefinidamente com a bola parada no lugar.
        #
        # Velocidade da bola e o unico sinal confiavel: empurrao fica em torno de
        # 1500 mm/s, chute passa de 5000.
        if self.estado.last_kicker_id is None:
            vel_bola = hypot(float(getattr(self.ball, "velocity_x", 0.0) or 0.0),
                             float(getattr(self.ball, "velocity_y", 0.0) or 0.0))
            if vel_bola > 2000.0:
                for c in comandos:
                    if getattr(c, "kick", 0.0) > 0.0:
                        self.estado.registrar_chute(c.robot_id)
                        break

        return TaskStatus.SUCCESS, comandos


class TheirFreekick(_BaseFreekick):
    """Cobranca do adversario: bloquear a linha de tiro e respeitar a distancia."""

    def __init__(self, ally_robots, ball, on_positive_half, enemy_robots=None,
                 estado=None, medidas=None, last_kicker_id=None,
                 last_kick_time=None):
        super().__init__(ally_robots, ball, on_positive_half, enemy_robots,
                         estado, medidas, last_kicker_id, last_kick_time)
        self.name = "OurDefense"
        self.angle = self.gk_angle

    # P12: o antigo _go_to_ball era um 'pass' que ninguem chamava. Removido.

    def _posicao_de_bloqueio(self, ordem, total):
        """Sobre a linha bola->nosso gol, respeitando os 500 mm da regra 5.3.3.

        Antes todos os robos recebiam exatamente o mesmo ponto medio entre a bola
        e o gol - empilhavam-se no mesmo lugar. Agora o primeiro fica na linha e
        os demais se abrem em leque, cobrindo os angulos de chute.
        """
        bx, by = self.ball.position_x, self.ball.position_y
        gx, gy = self.own_goal.x, self.own_goal.y
        dx, dy = gx - bx, gy - by
        n = hypot(dx, dy) or 1.0
        ux, uy = dx / n, dy / n
        px, py = -uy, ux

        recuo = max(DIST_MIN_ADVERSARIO + RAIO_ROBO, min(n * 0.45, 1800.0))
        if ordem == 0:
            desloc = 0.0
        else:
            lado = 1.0 if ordem % 2 == 1 else -1.0
            desloc = lado * (350.0 + 300.0 * ((ordem - 1) // 2))

        x = bx + ux * recuo + px * desloc
        y = by + uy * recuo + py * desloc
        return self._dentro_do_campo(x, y)

    def execute(self):
        if self._check_timeout():
            return TaskStatus.SUCCESS, self._parada_segura("tempo esgotado")
        if not self.ally_robots:
            return TaskStatus.RUNNING, []

        comandos = []

        if 0 in self.ally_robots:
            comandos.append(
                GoalkeeperKickoff().execute(
                    self.gk_target, self.angle,
                    ally_ids=list(self.ally_robots),
                    enemy_ids=list(self.enemy_robots),
                )
            )

        de_linha = self._ids_de_linha()
        for ordem, rid in enumerate(de_linha):
            alvo_x, alvo_y = self._posicao_de_bloqueio(ordem, len(de_linha))
            cmd = self.skills_factory.move_with_angle(
                robot_id=rid, target_x=alvo_x, target_y=alvo_y,
                vel_x=0.0, vel_y=0.0,
                angle=atan2(self.ball.position_y - alvo_y,
                            self.ball.position_x - alvo_x),
            )
            self._obstaculos(cmd, rid, evitar_bola=True)
            cmd.deactivate_kick()
            comandos.append(cmd)

        return TaskStatus.SUCCESS, comandos
