from strategy.skills.skills import Skills
from strategy.skills import geometria, posicionamento
from strategy.skills.bola import prever_direcao_chute, cruzamento_da_bola
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

PODE_SAIR_DA_LINHA = False


class Goalkeeper:
    def __init__(self, gk_info, ball, on_positive_half,
                 ally_robots=None, enemy_robots=None):
        self.name = "Goalkeeper"
        self.skills_factory = Skills("Movement")
        self.gk = gk_info
        self.ball = ball
        self.padding = 100
        self.on_positive_half = on_positive_half
        # Para SAIR JOGANDO: sem os companheiros e os adversarios ele nao tem
        # como escolher para quem tocar. Opcionais para nao quebrar quem ja
        # constroi o goleiro sem eles.
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
        """y onde um chute (já feito ou iminente) cruza o gol, ou None."""
        y = cruzamento_da_bola(self.ball, goal_x)
        if y is not None:
            return y
        prev = self._previsao_chute_iminente(goal_x)
        return prev["target_y"] if prev is not None else None

    def execute(self, goal_position: Vector2D, ball: Vector2D):
        """
        Quando a bola está na área do gol, o goleiro segue a lógica de ataque:
        - posiciona-se atrás da bola (considerando a direção de empurrar para fora do gol)
        - quando estiver próximo o suficiente, empurra a bola para fora da área

        Caso contrário, posiciona-se no centro da meta alinhado com a posição y da bola.
        """
        # ângulo do robô olhando para a bola (usado para orientação)
        angle = self._get_ball_angle()

        # Se a bola estiver na área do gol (uso goal_position para suportar ambos os lados)
        # definimos a área em função do lado do gol recebido.
        #
        # UMA conta so, em _bola_na_area. Antes a mesma condicao estava escrita
        # duas vezes - aqui e no _ball_in_goal_area, marcado como 'deprecated' -
        # com os mesmos numeros errados nas duas. Duas copias da mesma regra e
        # como elas divergem; ver o motivo do ararabots.sh ser ferramenta unica.
        goal_x = goal_position.x
        in_area = self._bola_na_area(goal_x)

        y_chute = self._y_do_chute(goal_x)
        if y_chute is not None:
            robot_command = self.skills_factory.move_with_angle(
                robot_id=0,
                target_x=goal_x,
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

            bx, by = ball.position_x, ball.position_y

            # direção para empurrar: do centro da meta para a bola (ou seja, do gol para fora)
            dx = bx - goal_position.x
            dy = by - goal_position.y
            norm = hypot(dx, dy) or 1.0
            ux, uy = dx / norm, dy / norm

            # ponto "atrás" da bola para se posicionar antes de empurrar
            behind_dist = 70.0
            stage_x = bx - ux * behind_dist
            stage_y = by - uy * behind_dist

            # obter posição atual do goleiro
            rx = getattr(self.gk, "position_x", None)
            ry = getattr(self.gk, "position_y", None)

            dist_to_stage = hypot(rx - stage_x, ry - stage_y) if rx is not None else float("inf")
            dist_to_ball = hypot(rx - bx, ry - by) if rx is not None else float("inf")

            # se estiver próximo o suficiente, faça o push; senão aproxime-se e alinhe-se atrás
            # ARMA POR PROXIMIDADE, como os robos de linha e como o
            # adversario. Observado no replay: "o goleiro esta encostado na bola
            # e nao chuta de jeito nenhum". O ramo de passe/saida abaixo so era
            # alcancado dentro de 230 mm do PONTO DE ENCAIXE, nao da bola - e
            # com a bola colada nele esse ponto fica atras do proprio corpo.
            if dist_to_ball < 230.0 or dist_to_stage < 230.0:
                # SAIR JOGANDO: se ha companheiro com linha livre, toca nele.
                #
                # O empurrao para fora da area continua sendo o plano B, para
                # quando nao houver ninguem livre. Mas empurrar devolve a bola
                # solta no nosso campo, que e de onde o adversario chuta.
                alvo = self._companheiro_livre()
                if alvo is not None:
                    ax, ay = alvo
                    ang_passe = atan2(ay - by, ax - bx)
                    dist = hypot(ax - bx, ay - by)
                    target_x = bx + (ax - bx) / (dist or 1.0) * 250.0
                    target_y = by + (ay - by) / (dist or 1.0) * 250.0
                    robot_command = self.skills_factory.move_with_angle(
                        robot_id=0,
                        target_x=target_x,
                        target_y=target_y,
                        vel_x=0.0,
                        vel_y=0.0,
                        angle=ang_passe,
                    )
                    robot_command.field_border = True
                    robot_command.ally_ids = []
                    # FORCA PELA DISTANCIA: o atrito do grSim e abrupto - 3 m/s
                    # percorre 1633 mm, 5 m/s percorre 1803, 6 m/s percorre
                    # 4220. Passe curto com forca de gol atravessa o receptor.
                    robot_command.kick = 2.5 if dist < 2500.0 else 4.0
                    return robot_command

                # SEM COMPANHEIRO LIVRE: CHUTA PARA O ESPACO A FRENTE.
                #
                # Pedido do Felipe: "o goleiro tinha a bola dominada e mesmo
                # assim nao chutou". Empurrar 220 mm deixa a bola na nossa area,
                # que e exatamente de onde eles chutam - medimos 267 toques com
                # a bola parada em x=-4150.
                #
                # Sem ninguem para receber, ele manda a bola para o espaco mais
                # aberto a frente. Nao e um passe, e uma saida: tirar a bola da
                # area vale mais que mante-la la.
                #
                # DIVISION B, regra do Aimless Kick: se a bola cruzar o meio e
                # sair pela linha de fundo deles sem tocar em ninguem, a falta e
                # DELES. Por isso o alvo e a meia-altura do campo e nao o fundo,
                # e a forca e media - queremos que ela PARE no campo deles.
                espaco = self._espaco_livre_a_frente()
                if espaco is not None:
                    ex, ey = espaco
                    ang_saida = atan2(ey - by, ex - bx)
                    robot_command = self.skills_factory.move_with_angle(
                        robot_id=0,
                        target_x=bx + (ex - bx) * 0.15,
                        target_y=by + (ey - by) * 0.15,
                        vel_x=0.0, vel_y=0.0, angle=ang_saida,
                    )
                    robot_command.field_border = True
                    robot_command.ally_ids = []
                    robot_command.kick = 4.5
                    return robot_command

                push_dist = 220.0
                target_x = bx + ux * push_dist
                target_y = by + uy * push_dist

                robot_command = self.skills_factory.move_with_angle(
                    robot_id=0,
                    target_x=target_x,
                    target_y=target_y,
                    vel_x=0.0,
                    vel_y=0.0,
                    angle=angle,
                )

                # permitir interação com a bola para empurrar
                robot_command.field_border = True
                robot_command.ally_ids = []
                try:
                    robot_command.deactivate_kick()
                except Exception:
                    pass

                return robot_command
            else:
                # mover para o ponto de alinhamento atrás da bola
                robot_command = self.skills_factory.move_with_angle(
                    robot_id=0,
                    target_x=stage_x,
                    target_y=stage_y,
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

        robot_command = self.skills_factory.move_with_angle(
            robot_id=0,
            target_x=goal_position.x,
            target_y=max(-400, min(400, ball.position_y)),
            vel_x=0.0,
            vel_y=0.0,
            angle=angle,
        )
        robot_command.field_border = True
        return robot_command