from new_movement.entities.States import Vector2D
import os

from math import atan2, hypot

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


class Atack:
    def __init__(self, ally_robots, enemy_robots, ball, on_positive_half):
        self.name = "OurAtack"
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
        push_dist = 220.0
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



    def execute(self):
        atacante = eleger_atacante(self.ally_robots, self.ball)
        if os.environ.get("DIAG_JOGO"):
            print("[JG] tatica=Atack half=%s" % self.on_positive_half, flush=True)
        robots_commands = []

        # comportamento do goleiro: usar a classe Goalkeeper para remover a bola da área caso necessário
        if 0 in self.ally_robots:
            gk_info = self.ally_robots[0]
            gk = Goalkeeper(gk_info, self.ball, self.on_positive_half)
            robots_commands.append(gk.execute(self.gk_target, self.ball))

        for robot_id_, _ in self.ally_robots.items():
            if robot_id_ == 0:
                continue

            # ELEICAO TAMBEM NO ATAQUE - era so na defesa, e foi a causa da
            # variancia que nos custou uma rodada inteira de trabalho.
            #
            # MEDIDO em 6 execucoes, contando os ciclos de cada tatica:
            #     rep1 ATACA 237 / DEFENDE 12    rep4 ATACA   0 / DEFENDE 251
            #     rep2 ATACA 240 / DEFENDE  9    rep5 ATACA 180 / DEFENDE  70
            #     rep3 ATACA 198 / DEFENDE 52    rep6 ATACA 236 / DEFENDE   7
            #
            # A arvore roda ATAQUE na maior parte do tempo, e toda a correcao
            # anterior (eleicao, empurrao, chute) tinha ido para a DEFESA. As
            # execucoes "boas" eram as que calharam de cair na defesa - a
            # variancia nao estava na geometria, e sim em QUAL tatica executava.
            #
            # Sem eleicao, TODOS os robos de linha chamavam _go_to_goal e
            # disputavam a mesma bola.
            if robot_id_ == atacante:
                command = self._go_to_goal(robot_id=robot_id_)
            else:
                command = self._cobertura(robot_id_, ordem_cob)
                ordem_cob += 1

            robots_commands.append(command)

        return robots_commands


class Defense:
    def __init__(self, ally_robots, ball, on_positive_half):
        self.name = "OurDefense"
        self.skills_factory = Skills("Movement")
        self.goal_center = CenterGoal()
        self.on_positive_half = on_positive_half
        self.ally_robots = ally_robots
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

    def execute(self):
        if os.environ.get("DIAG_JOGO"):
            print("[JG] tatica=Defense half=%s" % self.on_positive_half, flush=True)
        atacante = eleger_atacante(self.ally_robots, self.ball)
        robots_commands = []

        if 0 in self.ally_robots:
            gk_info = self.ally_robots[0]
            gk = Goalkeeper(gk_info, self.ball, self.on_positive_half)
            robots_commands.append(gk.execute(self.gk_target, self.ball))

        for robot_id_, _ in self.ally_robots.items():
            if robot_id_ == 0:
                continue

            ball_pos = Vector2D(self.ball.position_x, self.ball.position_y)
            goal_pos = (
                self.goal_center.GOAL_POSITIVE
                if self.on_positive_half
                else self.goal_center.GOAL_NEGATIVE
            )
            # O ELEITO VAI A BOLA; o resto continua cobrindo.
            #
            # Ver eleger_atacante para o impasse que isto quebra. Dois detalhes
            # que NAO sao decorativos:
            #
            #  - 'ball = False' para o eleito. Com a bola como obstaculo o
            #    planejador desvia dela e o robo orbita sem nunca toca-la. Era
            #    esse o motivo de a bola nao sair do lugar.
            #  - o corpo aponta na direcao GOL NOSSO -> BOLA, ou seja, para o
            #    campo de ataque: quem chega virado para tras empurra a bola
            #    para o nosso proprio gol. E a mesma licao do _atras_da_bola.
            kick_forca = 0.0
            if robot_id_ == atacante:
                # ALVO ALEM DA BOLA, e nao a bola.
                #
                # MEDIDO: mirando a PROPRIA bola, o eleito parou a 178 mm dela
                # (mediana de 3 execucoes) e a posse foi a ZERO - pior que os
                # 55 mm que o esbarrao acidental dava antes. Com o erro de
                # posicao indo a zero junto com a chegada, o robo estaciona
                # antes de encostar.
                #
                # E o mesmo P4 da cobranca de falta, e a mesma solucao do
                # AVANCO_RETO: pedir um ponto ALEM da bola faz o robo seguir
                # pressionando em vez de frear em cima dela.
                #
                # ATRAS DA BOLA (250 mm), do lado do NOSSO gol - e nao alem
                # dela.
                #
                # Mirar 400 mm ALEM piorou: d_min 178 -> 323 mm, com dispersao
                # enorme (702, 323, 97). Faz sentido: sem uma fase de contorno,
                # o robo aborda a bola de qualquer angulo, e um alvo do outro
                # lado dela o faz atravessar de trave a trave.
                #
                # Atras da bola ele fica POSICIONADO para empurrar na direcao
                # certa. E o ponto de encaixe da cobranca, com a mesma distancia
                # (DIST_ATRAS_DA_BOLA = 250) que se mostrou boa la.
                dgx = ball_pos.x - goal_pos.x
                dgy = ball_pos.y - goal_pos.y
                n = hypot(dgx, dgy) or 1.0
                ux, uy = dgx / n, dgy / n

                # DUAS FASES: primeiro ficar ATRAS da bola, depois EMPURRAR.
                #
                # So ir ao ponto atras da bola nao faz a bola andar - medido:
                # o eleito encostava (91 mm) e a bola ficava parada, x maximo
                # -1300 em 25 s. Faltava a segunda metade.
                #
                # E a mesma divisao que a cobranca de falta usa (encaixar ->
                # empurrar), reduzida ao essencial. A condicao de troca e
                # geometrica e nao guarda estado:
                #   - 'proj' e o quanto o robo esta a frente da bola no eixo do
                #     ataque. Negativo = atras dela, que e de onde se empurra.
                #   - 'lat' e o afastamento da linha gol_nosso -> bola.
                #
                # Estando atras e alinhado, o alvo passa para ALEM da bola e o
                # robo a leva junto. Fora disso, volta a se posicionar.
                r_info = self.ally_robots[robot_id_]
                rx = r_info.position_x - ball_pos.x
                ry = r_info.position_y - ball_pos.y
                proj = rx * ux + ry * uy
                lat = abs(-rx * uy + ry * ux)

                if proj < -60.0 and lat < 120.0:
                    # atras e alinhado: EMPURRA na direcao do ataque
                    defend_x = ball_pos.x + ux * 400.0
                    defend_y = ball_pos.y + uy * 400.0
                    # E CHUTA, se ja esta ao alcance do chutador.
                    #
                    # Sem isto a bola so era empurrada: medimos pico de
                    # 1746 mm/s em todas as execucoes, quando um chute real
                    # sai a 5000+. Empurrao nao tira a bola do nosso campo -
                    # sao 3 metros de conducao, e a cobranca de falta ja
                    # mostrou que empurrao nao sobrevive a contato lateral.
                    #
                    # A geometria exigida e a do grSim (robot.cpp:128), a mesma
                    # que a cobranca usa: a bola perto da placa e dentro da
                    # largura dela. Aqui basta a distancia, porque 'lat' ja
                    # garantiu o alinhamento e o corpo ja aponta para o ataque.
                    #
                    # FORCA: a cobranca mediu que abaixo de 6 m/s a bola morre
                    # antes de cruzar o campo (3,0 m/s percorre 1633 mm; 6,0
                    # percorre 4220). O activate_kick() da Skills usa 1,5, que
                    # nao tira a bola de perto - por isso o valor vai direto.
                    dist_bola = hypot(rx, ry)
                    if dist_bola < FORCA_CHUTE_ALCANCE:
                        kick_forca = FORCA_CHUTE
                else:
                    # ainda nao: vai para o ponto atras da bola
                    defend_x = ball_pos.x - ux * 250.0
                    defend_y = ball_pos.y - uy * 250.0
                ang_robo = atan2(uy, ux)
                bola_obstaculo = False
            else:
                # APOIO ATRAS, e nao a frente. TENTADO O CONTRARIO E MEDIDO.
                #
                # A ideia era obvia: o time nunca entrava no campo de ataque
                # (zero quadros alem de x=3500), entao ponha o nao eleito
                # ADIANTE da bola, oferecendo linha de passe - a mesma geometria
                # do _posicao_de_apoio da cobranca.
                #
                # NAO FUNCIONOU, duas vezes, e piorou o que ja andava:
                #   apoio a 1200 mm / 900 de abertura: chute 5525 -> 1536 mm/s,
                #     x maximo 2038 -> -1147
                #   apoio a 1800 / 1600 (para sair do corredor): chute 1159,
                #     so 1 de 3 execucoes chutou
                #
                # A leitura honesta: com apenas DOIS robos de linha, mandar um
                # para a frente deixa o eleito sozinho e ainda coloca um corpo
                # no caminho por onde ele precisa empurrar. Posicionamento
                # ofensivo provavelmente so se paga com tres ou mais robos de
                # linha - vale retomar quando o cenario crescer.
                defend_x = (ball_pos.x + goal_pos.x) / 2.0
                defend_y = (ball_pos.y + goal_pos.y) / 2.0
                ang_robo = self.angle
                bola_obstaculo = True


            robot_command = self.skills_factory.move_with_angle(
                robot_id=robot_id_,
                target_x=defend_x,
                target_y=defend_y,
                vel_x=0.0,
                vel_y=0.0,
                angle=ang_robo,
            )

            # SONDA DO ELEITO (DIAG_JOGO=1): o que separa uma execucao boa de
            # uma ruim. Entre 1 e 3 execucoes em 3 chutam, e nao sabemos por que.
            if os.environ.get("DIAG_JOGO") and robot_id_ == atacante:
                print("[JG] eleito=%d proj=%.0f lat=%.0f d=%.0f kick=%.1f "
                      "fase=%s bola=(%.0f,%.0f)"
                      % (robot_id_, proj, lat, hypot(rx, ry), kick_forca,
                         "EMPURRA" if (proj < -60.0 and lat < 120.0) else "posiciona",
                         ball_pos.x, ball_pos.y), flush=True)
            robot_command.kick = kick_forca
            robot_command.ball = bola_obstaculo
            robot_command.field_border = True
            robot_command.penalty_area = True

            robots_commands.append(robot_command)

        return robots_commands
