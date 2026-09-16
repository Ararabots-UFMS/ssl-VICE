from strategy.skills.skills import Skills
from utils.math_util import Vector2D
from math import atan2, hypot


# O companheiro precisa estar ao menos isto a frente da bola para receber o
# passe do goleiro. Menos que isso e passe lateral dentro da propria area.
AVANCO_MINIMO_PASSE = 600.0


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
        from strategy.tatics.running import linha_livre

        bx, by = self.ball.position_x, self.ball.position_y
        sentido = -1.0 if self.on_positive_half else 1.0
        # meia altura do campo, para a bola PARAR antes da linha de fundo deles
        alvo_x = 1000.0 * sentido
        melhor, melhor_folga = None, -1.0
        for alvo_y in (-2000.0, -1000.0, 0.0, 1000.0, 2000.0):
            dx, dy = alvo_x - bx, alvo_y - by
            n = hypot(dx, dy) or 1.0
            if not linha_livre(bx, by, dx / n, dy / n, self.enemy_robots):
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
        from strategy.tatics.running import linha_livre

        bx, by = self.ball.position_x, self.ball.position_y
        sentido = -1.0 if self.on_positive_half else 1.0
        melhor, melhor_av = None, None
        for rid, r in self.ally_robots.items():
            if rid == 0:
                continue
            dx, dy = r.position_x - bx, r.position_y - by
            n = hypot(dx, dy) or 1.0
            if n < 400.0:
                continue          # colado demais: o toque nao sai
            if not linha_livre(bx, by, dx / n, dy / n, self.enemy_robots):
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

    def _ball_in_goal_area(self) -> bool:
        # deprecated: keep for compatibility but prefer using goal_position-aware check in execute
        if self.on_positive_half:
            if (
                self.ball.position_x > 1750.0 - self.padding
                and abs(self.ball.position_y) < 700.0 - self.padding
            ):
                return True
        else:
            if (
                self.ball.position_x < -1750.0 + self.padding
                and abs(self.ball.position_y) < 700.0 - self.padding
            ):
                return True

        return False

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
        goal_x = goal_position.x
        if goal_x > 0:
            in_area = (
                self.ball.position_x > 1750.0 - self.padding
                and abs(self.ball.position_y) < 700.0 - self.padding
            )
        else:
            in_area = (
                self.ball.position_x < -1750.0 + self.padding
                and abs(self.ball.position_y) < 700.0 - self.padding
            )

        if in_area:
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
