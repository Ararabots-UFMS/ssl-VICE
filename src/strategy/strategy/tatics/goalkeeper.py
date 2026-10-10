from strategy.skills.skills import Skills
from strategy.skills import geometria, posicionamento
from strategy.skills.bola import prever_direcao_chute, cruzamento_da_bola, GOL_MEIA_LARGURA
from utils.math_util import Vector2D
from math import atan2, hypot


# O companheiro precisa estar ao menos isto a frente da bola para receber o
# passe. A constante MORA NA CAMADA DE SKILLS (skills/posicionamento.py): o
# mesmo valor, com o mesmo motivo, estava escrito aqui e em tatics/running.py.
AVANCO_MINIMO_PASSE = posicionamento.AVANCO_MINIMO_PASSE

# AREA DE DEFESA — Division B, em milimetros.
#
# Fonte: o proprio grSim, que arbitra o campo do teste. O ~/.grsim.xml traz,
# para a Division B, 'Penalty width = 2' e 'Penalty depth = 1' (metros) - logo
# 2000 mm de largura por 1000 de profundidade. Com a linha de gol em x = +-4500,
# a nossa area e |y| <= 1000 e x a ate 1000 mm da linha.
#
# Nao repetir estes numeros em outro lugar. Ver _bola_na_area.
AREA_PROFUNDIDADE = 1000.0
AREA_MEIA_LARGURA = 1000.0

# True: bola DENTRO da area -> o goleiro vai na bola e chuta (ver _atacar_a_bola).
# Bola FORA da area -> ele fecha o angulo (ver _posicao_fechando_angulo).
PODE_SAIR_DA_LINHA = True

# FECHAR O ANGULO
# goal_position.x e onde o goleiro FICA (-4335), nao a linha de gol: os postes
# ficam na linha REAL.
CAMPO_MEIO_COMPRIMENTO = 4500.0
ADIANTAMENTO_MAX = 900.0          # quanto ele avanca da linha dele, no maximo
DIST_ATACANTE_MIN = 250.0         # atacante a menos disto da bola -> avanco total
DIST_ATACANTE_MAX = 1000.0        # atacante a mais disto da bola  -> sem avanco
DIST_BOLA_GOL_MIN = 1200.0        # bola a menos disto do gol      -> avanco total
DIST_BOLA_GOL_MAX = 2500.0        # bola a mais disto do gol       -> sem avanco

# IR NA BOLA E CHUTAR
# O goleiro se alinha ATRAS da bola, no eixo goleiro->alvo do chute, e so entao
# avanca atravessando a bola. Alinhar atras evita empurra-la para a propria meta.
DIST_ENCAIXE = 180.0              # ponto de encaixe: tanto atras da bola
CORREDOR_LATERAL = 90.0           # |desvio lateral| maximo para ja poder bater
CORREDOR_FUNDO = 350.0            # ate quanto atras da bola o corredor vale
ATRAVESSAR_BOLA = 250.0           # o alvo do movimento fica tanto ALEM da bola


def _limita(v, lo, hi):
    return max(lo, min(hi, v))


class Goalkeeper:
    def __init__(self, gk_info, ball, on_positive_half,
                 ally_robots=None, enemy_robots=None):
        self.name = "Goalkeeper"
        self.skills_factory = Skills("Movement")
        self.gk = gk_info
        self.ball = ball
        self.padding = 100
        self.on_positive_half = on_positive_half
        # Para SAIR JOGANDO e para FECHAR O ANGULO: sem os companheiros e os
        # adversarios ele nao tem como escolher para quem tocar nem saber se ha
        # atacante perto. Opcionais para nao quebrar quem ja constroi o goleiro
        # sem eles. SE O GOLEIRO FOR REUTILIZADO ENTRE CICLOS, estes campos (e
        # self.gk) precisam ser atualizados a cada ciclo.
        self.ally_robots = ally_robots or {}
        self.enemy_robots = enemy_robots or {}

    def _espaco_livre_a_frente(self):
        """O ponto mais aberto no meio-campo, para tirar a bola da area.

        Varre alguns pontos a meia altura do campo e escolhe o que estiver mais
        longe de qualquer adversario. E o mesmo principio que o ITAndroids usa
        para mirar o gol - varrer e escolher o setor mais livre - aplicado aqui
        a saida de bola.
        """
        linha_livre = geometria.linha_livre

        bx, by = self.ball.position_x, self.ball.position_y
        sentido = -1.0 if self.on_positive_half else 1.0
        # meia altura do campo, para a bola PARAR antes da linha de fundo deles
        alvo_x = 1000.0 * sentido
        # A LINHA CONFERIDA E ATE O ALVO, nao ate um versor.
        #
        # 'linha_livre(ox, oy, dx_, dy_, inimigos)' espera as COORDENADAS DO
        # DESTINO em dx_/dy_ (ver tatics/running.py). Passavamos 'dx/n, dy/n',
        # que e um VERSOR: o segmento conferido ia da bola ate um ponto a ~1 mm
        # da origem do campo. A checagem existia e nao conferia a linha pedida.
        #
        # MEDIDO na linha de base: com a bola na nossa area, o goleiro passou a
        # engajar (A1), mas a escolha de PARA ONDE mandar era feita sobre esta
        # conta errada - e a bola ficou no nosso campo 99,1% do tempo.
        melhor, melhor_folga = None, -1.0
        for alvo_y in (-2000.0, -1000.0, 0.0, 1000.0, 2000.0):
            if not linha_livre(bx, by, alvo_x, alvo_y, self.enemy_robots):
                continue
            folga = min([hypot(e.position_x - alvo_x, e.position_y - alvo_y)
                         for e in self.enemy_robots.values()] or [9999.0])
            if folga > melhor_folga:
                melhor, melhor_folga = (alvo_x, alvo_y), folga
        return melhor

    def _companheiro_livre(self):
        """Para quem tocar: o companheiro com a linha limpa, o mais adiantado.

        POR QUE ISTO EXISTE
        -------------------
        O goleiro tinha 'deactivate_kick()' em TODOS os ramos - ele nunca
        chutava. Observado em replay: ele recupera a bola e fica trocando toques
        com o atacante adversario, 12 toques alternados na mesma posicao, ate
        alguem tirar. Recuperar a bola e devolve-la ao ataque deles.

        Escolhe o mais adiantado entre os que tem linha livre, porque tocar para
        tras so adia o problema.
        """
        linha_livre = geometria.linha_livre

        bx, by = self.ball.position_x, self.ball.position_y
        sentido = -1.0 if self.on_positive_half else 1.0
        melhor, melhor_av = None, None
        for rid, r in self.ally_robots.items():
            if rid == 0:
                continue
            n = hypot(r.position_x - bx, r.position_y - by)
            if n < 400.0:
                continue          # colado demais: o toque nao sai
            # A LINHA CONFERIDA E ATE O COMPANHEIRO. Ver _espaco_livre_a_frente:
            # passavamos um versor onde linha_livre espera o DESTINO, entao esta
            # condicao nunca olhou para a linha bola->companheiro.
            if not linha_livre(bx, by, r.position_x, r.position_y,
                               self.enemy_robots):
                continue
            # NUNCA PARA TRAS.
            #
            # Sem isto ele escolhe "o menos ruim" quando todos estao atras da
            # bola - e medimos o goleiro chutando a 5398 mm/s na direcao da
            # PROPRIA meta, 555 mm, em x=-4118. Quase gol contra.
            #
            # Se ninguem esta a frente, o plano B (empurrar para fora da area)
            # e melhor que um passe para dentro.
            avanco = r.position_x * sentido
            if avanco <= bx * sentido + AVANCO_MINIMO_PASSE:
                continue
            if melhor_av is None or avanco > melhor_av:
                melhor, melhor_av = (r.position_x, r.position_y), avanco
        return melhor

    def _get_ball_angle(self) -> float:
        dx = self.ball.position_x - self.gk.position_x
        dy = self.ball.position_y - self.gk.position_y
        angle = atan2(dy, dx)
        return angle

    def _bola_na_area(self, goal_x: float) -> bool:
        """A bola esta na NOSSA area de defesa?

        O QUE ESTAVA ERRADO, e o que medimos
        ------------------------------------
        A condicao era 'x > 1750 - padding' com '|y| < 700 - padding'. Esses
        numeros vem do campo antigo de +-2250 (meia-largura de SSL-EL), o mesmo
        erro de geometria que o CenterGoal ja teve e que MedidasCampo (em
        tatics/freekick.py) foi criada para nao deixar acontecer de novo. O
        CenterGoal foi corrigido para +-4500; isto aqui ficou para tras.

        Eu previ que o efeito seria o goleiro ABANDONAR a meta, porque em x a
        condicao antiga e permissiva demais: com o gol em -4500, 'x < -1650'
        libera a saida a 2850 mm do gol. MEDIDO nos 6 replays da linha de base:
        o goleiro NUNCA passou de x = -3500. A previsao estava errada.

        O defeito real esta no OUTRO eixo. Comparando a condicao antiga com a
        area verdadeira da Division B em 8472 quadros:

            falso positivo (age fora da area)        179 quadros   2,1%
            falso negativo (NAO age dentro dela)    1373 quadros  16,2%

        O '|y| < 600' e restritivo demais: a area tem 2000 mm de largura, ou
        seja |y| < 1000. Com a bola na area entre 600 e 1000 mm do centro, o
        goleiro ficava parado na linha assistindo. Numa das execucoes isso
        durou 987 quadros - 16 dos 24 segundos de partida.

        AS MEDIDAS, e a fonte delas: o proprio grSim arbitra o campo do teste, e
        o ~/.grsim.xml declara para a Division B 'Penalty width = 2' e
        'Penalty depth = 1'. Logo a area tem 2000 mm de largura por 1000 de
        profundidade: |y| <= 1000, e x a ate 1000 mm da linha de gol.

        'padding' passa a ser FOLGA que AUMENTA a area, nao que a encolhe: o
        goleiro comecar a tratar a bola um pouco antes de ela entrar e barato,
        e chegar tarde e caro.
        """
        if goal_x > 0:
            dentro_x = self.ball.position_x > goal_x - AREA_PROFUNDIDADE - self.padding
        else:
            dentro_x = self.ball.position_x < goal_x + AREA_PROFUNDIDADE + self.padding
        return dentro_x and abs(self.ball.position_y) < AREA_MEIA_LARGURA + self.padding

    def _previsao_chute_iminente(self, goal_x: float):
        """Checa se um atacante adversario provavelmente vai chutar ao gol."""
        melhor = None
        if not self.enemy_robots:
            return None

        for enemy in self.enemy_robots.values():
            prev = prever_direcao_chute(enemy, self.ball, goal_x)
            if prev is None:
                continue
            if melhor is None or prev["tempo"] < melhor["tempo"]:
                melhor = prev
        return melhor

    def _y_do_chute(self, goal_x: float):
        """(y, em_voo) onde um chute cruza o gol, ou None."""
        y = cruzamento_da_bola(self.ball, goal_x)
        if y is not None:
            return y, True
        prev = self._previsao_chute_iminente(goal_x)
        return (prev["target_y"], False) if prev is not None else None

    @staticmethod
    def _sinal(goal_x):
        return 1.0 if goal_x > 0 else -1.0

    def _fator_de_avanco(self, goal_x):
        """0..1: quanto fechar o angulo.

        1 = atacante colado na bola E bola perto do gol; 0 = qualquer um dos dois
        longe. E continuo (sem degrau), para o goleiro nao ficar indo e voltando
        quando a distancia do atacante oscila perto do limite.
        """
        if not self.enemy_robots:
            return 0.0
        s = self._sinal(goal_x)
        bx, by = self.ball.position_x, self.ball.position_y
        poste_x = s * CAMPO_MEIO_COMPRIMENTO
        if (poste_x - bx) * s <= 1.0:      # bola na linha de gol ou atras dela
            return 0.0
        d_atac = min(hypot(e.position_x - bx, e.position_y - by)
                     for e in self.enemy_robots.values())
        f_atac = _limita((DIST_ATACANTE_MAX - d_atac)
                         / (DIST_ATACANTE_MAX - DIST_ATACANTE_MIN), 0.0, 1.0)
        d_gol = hypot(poste_x - bx, by)
        f_bola = _limita((DIST_BOLA_GOL_MAX - d_gol)
                         / (DIST_BOLA_GOL_MAX - DIST_BOLA_GOL_MIN), 0.0, 1.0)
        return f_atac * f_bola

    def _posicao_fechando_angulo(self, goal_x):
        """(x, y) sobre a BISSETRIZ do angulo de chute, avancado da linha.

        Sem atacante perto ou com a bola longe, devolve o ponto de sempre: na
        linha, acompanhando o y da bola (limitado a +-400).
        """
        s = self._sinal(goal_x)
        bx, by = self.ball.position_x, self.ball.position_y
        padrao = (goal_x, _limita(by, -400.0, 400.0))

        f = self._fator_de_avanco(goal_x)
        if f <= 0.01:
            return padrao

        poste_x = s * CAMPO_MEIO_COMPRIMENTO
        ax, ay = poste_x - bx, GOL_MEIA_LARGURA - by
        cx, cy = poste_x - bx, -GOL_MEIA_LARGURA - by
        na, nc = hypot(ax, ay) or 1.0, hypot(cx, cy) or 1.0
        dx, dy = ax / na + cx / nc, ay / na + cy / nc
        nd = hypot(dx, dy)
        if nd < 1e-6 or dx * s <= 0.1 * nd:
            return padrao
        dx, dy = dx / nd, dy / nd

        alvo_x = goal_x - s * ADIANTAMENTO_MAX * f
        t = (alvo_x - bx) / dx
        if t <= 0:
            return padrao
        return alvo_x, _limita(by + t * dy, -400.0, 400.0)

    def _x_do_chute(self, goal_x, em_voo):
        """Profundidade do goleiro quando ha chute a gol.

        Chute iminente: fecha o angulo, como no resto. Chute JA EM VOO: o atacante
        ja se afastou da bola, entao o fator cairia e o goleiro recuaria no pior
        momento - mantem a profundidade atual e so anda de lado.
        """
        s = self._sinal(goal_x)
        if em_voo:
            rx = getattr(self.gk, "position_x", None)
            if rx is not None:
                lo, hi = sorted((goal_x, goal_x - s * ADIANTAMENTO_MAX))
                return _limita(rx, lo, hi)
            return goal_x
        return self._posicao_fechando_angulo(goal_x)[0]

    def _atacar_a_bola(self, goal_position):
        """Bola DENTRO da area: vai na bola e a tira dali.

        1. Escolhe PARA ONDE bater: companheiro livre (toque), senao o espaco
           mais aberto a frente (saida), senao empurra para longe do gol (sem
           chute).
        2. Alinha-se ATRAS da bola no eixo bola->alvo. Chegar de lado, ou pela
           frente, empurraria a bola para a propria meta.
        3. Dentro do corredor atras da bola, avanca ATRAVESSANDO-A, com o
           chute armado.

        DIVISION B, Aimless Kick: o alvo da saida e a meia-altura do campo, nao
        o fundo, e a forca e media - a bola deve PARAR no campo deles.
        """
        bx, by = self.ball.position_x, self.ball.position_y

        # 1. PARA ONDE BATER
        forca = None
        alvo = self._companheiro_livre()
        if alvo is not None:
            ax, ay = alvo
            # FORCA PELA DISTANCIA: o atrito do grSim e abrupto - 3 m/s
            # percorre 1633 mm, 5 m/s percorre 1803, 6 m/s percorre 4220.
            # Passe curto com forca de gol atravessa o receptor.
            forca = 2.5 if hypot(ax - bx, ay - by) < 2500.0 else 4.0
        else:
            alvo = self._espaco_livre_a_frente()
            if alvo is not None:
                ax, ay = alvo
                forca = 4.5
            else:
                # plano B: empurra para fora, longe do gol, sem chute
                dx, dy = bx - goal_position.x, by - goal_position.y
                n = hypot(dx, dy) or 1.0
                ax, ay = bx + dx / n * 1000.0, by + dy / n * 1000.0

        dist = hypot(ax - bx, ay - by) or 1.0
        ux, uy = (ax - bx) / dist, (ay - by) / dist
        angulo = atan2(uy, ux)

        # 2. ONDE ESTA O GOLEIRO EM RELACAO A BOLA, no eixo do chute
        rx = getattr(self.gk, "position_x", None)
        ry = getattr(self.gk, "position_y", None)
        pronto = False
        if rx is not None and ry is not None:
            ox, oy = rx - bx, ry - by
            ao_longo = ox * ux + oy * uy          # < 0: atras da bola
            lateral = -ox * uy + oy * ux
            pronto = (abs(lateral) <= CORREDOR_LATERAL
                      and -CORREDOR_FUNDO <= ao_longo <= 30.0)

        # 3. ALINHAR ou BATER
        if pronto:
            target_x = bx + ux * ATRAVESSAR_BOLA
            target_y = by + uy * ATRAVESSAR_BOLA
        else:
            target_x = bx - ux * DIST_ENCAIXE
            target_y = by - uy * DIST_ENCAIXE

        robot_command = self.skills_factory.move_with_angle(
            robot_id=0,
            target_x=target_x,
            target_y=target_y,
            vel_x=0.0,
            vel_y=0.0,
            angle=angulo,
        )
        robot_command.field_border = True
        robot_command.ally_ids = []
        if pronto and forca is not None:
            robot_command.kick = forca
        else:
            try:
                robot_command.deactivate_kick()
            except Exception:
                pass
        return robot_command

    def execute(self, goal_position: Vector2D, ball: Vector2D):
        """
        Prioridade:
        1. Chute a gol (ja em voo ou iminente): posiciona-se sobre o ponto de
           cruzamento, fechando o angulo se o chute ainda nao saiu.
        2. Bola DENTRO da area: vai na bola e chuta (_atacar_a_bola).
        3. Bola FORA da area: fecha o angulo na bissetriz, avancando da linha
           conforme o atacante e a bola estao perto (_posicao_fechando_angulo).
        """
        self.ball = ball

        # ângulo do robô olhando para a bola (usado para orientação)
        angle = self._get_ball_angle()

        # UMA conta so, em _bola_na_area (suporta os dois lados via goal_x).
        goal_x = goal_position.x
        in_area = self._bola_na_area(goal_x)

        chute = self._y_do_chute(goal_x)
        if chute is not None:
            y_chute, em_voo = chute
            robot_command = self.skills_factory.move_with_angle(
                robot_id=0,
                target_x=self._x_do_chute(goal_x, em_voo),
                target_y=max(-400.0, min(400.0, y_chute)),
                vel_x=0.0,
                vel_y=0.0,
                angle=angle,
            )
            robot_command.field_border = True
            robot_command.ally_ids = []
            try:
                robot_command.deactivate_kick()
            except Exception:
                pass
            return robot_command

        if in_area and PODE_SAIR_DA_LINHA:
            return self._atacar_a_bola(goal_position)

        alvo_x, alvo_y = self._posicao_fechando_angulo(goal_x)
        robot_command = self.skills_factory.move_with_angle(
            robot_id=0,
            target_x=alvo_x,
            target_y=alvo_y,
            vel_x=0.0,
            vel_y=0.0,
            angle=angle,
        )
        robot_command.field_border = True
        return robot_command