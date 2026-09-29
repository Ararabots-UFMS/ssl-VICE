import os
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from strategy.root import RootTree
from typing import Iterable
from system_interfaces.srv import SetOrientation, UpdateKick
from strategy.skills.skills import Skill


class Strategy(Node):
    def __init__(self, wait_for_service: bool = True):
        super().__init__("strategy_node")
        self.get_logger().info("Strategy node initialized")

        # MOVIMENTACAO NOVA (dev) ou ANTIGA (driver), escolhida por variavel.
        #
        # MOVIMENTO_NOVO=1 -> publica MovementCommandArray em
        #   'movement_manager/commands'; quem planeja e o planner_node, que
        #   ancora no estado da VISAO (o conserto do §6.3: o driver antigo
        #   replaneja a partir do setpoint anterior, nunca da posicao medida).
        # sem a variavel -> servico 'strategy_command' do driver, como antes.
        #
        # Os dois caminhos ficam lado a lado de proposito: e a unica forma de
        # medir a diferenca da movimentacao sem trocar de branch no meio do
        # lote. Orientacao e chute NAO mudam - continuam nos servicos do
        # pacote control, que a dev nao mexeu.
        # UM CAMINHO SO desde que a dev removeu o driver.
        #
        # 'StrategyCommand' e 'UpdateObstacle' sairam do system_interfaces junto
        # com o driver.py ("refactor: dropping the driver and renaming
        # new_movement to movement"), entao nao ha mais alternativa a manter.
        self.movimento_novo = True

        self.move_cli = None
        self.mov_pub = None
        if self.movimento_novo:
            from movement_interfaces.msg import MovementCommandArray, MovementCommand
            from movement_interfaces.srv import SetStaticObstacles, SetGoalKeeper
            self._MovementCommandArray = MovementCommandArray
            self._MovementCommand = MovementCommand
            self.mov_pub = self.create_publisher(
                MovementCommandArray, "movement_manager/commands", 10)
            self.estaticos_cli = self.create_client(
                SetStaticObstacles, "SetStaticObstacles")
            self.goleiro_cli = self.create_client(SetGoalKeeper, "SetGoalKeeper")
            self._SetStaticObstacles = SetStaticObstacles
            self._SetGoalKeeper = SetGoalKeeper
            self._configurou_manager = False
        else:
            self.move_cli = None   # driver removido pela dev
        self.orientation_cli = self.create_client(SetOrientation, "set_orientation")
        self.obstacle_cli = None   # driver removido pela dev
        self.kick_cli = self.create_client(UpdateKick, "update_kick")

        if wait_for_service:
            if self.movimento_novo:
                while not self.estaticos_cli.wait_for_service(timeout_sec=3.0):
                    self.get_logger().info('Aguardando serviço "SetStaticObstacles"...')
                while not self.goleiro_cli.wait_for_service(timeout_sec=3.0):
                    self.get_logger().info('Aguardando serviço "SetGoalKeeper"...')
            else:
                while not self.move_cli.wait_for_service(timeout_sec=3.0):
                    self.get_logger().info('Aguardando serviço "strategy_command"...')
            while not self.orientation_cli.wait_for_service(timeout_sec=3.0):
                self.get_logger().info('Aguardando serviço "set_orientation"...')
            # 'update_obstacles' e do DRIVER, que nao sobe no caminho novo.
            # Esperar por ele ali trava a inicializacao para sempre.
            while not self.movimento_novo and not self.obstacle_cli.wait_for_service(
                    timeout_sec=3.0):
                self.get_logger().info('Aguardando serviço "update_obstacles"...')
            while not self.kick_cli.wait_for_service(timeout_sec=3.0):
                self.get_logger().info('Aguardando serviço "kick_command"...')

        # CONFIGURA O MANAGER ANTES DE COMECAR A COMANDAR.
        #
        # Era feito preguicosamente, no primeiro _send_move - ou seja, as duas
        # chamadas de servico saiam NO MESMO CICLO da primeira publicacao de
        # alvos, correndo com ela. E o proprio comentario de _configurar_manager
        # avisa o que acontece nesse intervalo: sem _static_obstacles e
        # _goal_keeper_id preenchidos, o _ready_to_publish() do MovementManager
        # recusa, ele recebe os comandos e NAO publica alvo nenhum - sem erro,
        # sem log, sem nada.
        #
        # MEDIDO (sonda [LG], cenario 'jogo'): no ciclo 1, t=0,000 apos o
        # FORCE_START, a estrategia ja manda r1 para (-28,-7), que e a bola no
        # centro, com o robo a 1172 mm dela. E o robo anda 3 mm em 0,6 s. O
        # comando certo sai na hora certa e nao vira movimento.
        #
        # Aqui as duas chamadas acontecem na construcao, depois do
        # wait_for_service dos dois servicos logo acima - entao quando o primeiro
        # alvo for publicado o manager ja tem o que precisa.
        if self.movimento_novo:
            self._configurar_manager()

        self.timer = self.create_timer(0.1, self.run)
        self.root = RootTree("RootStrategy")

        # Ultimo alvo enviado por robo, para nao repedir a mesma coisa.
        self._ultimo_alvo: dict[int, tuple] = {}
        self._ultimo_envio: dict[int, float] = {}
        # Ultimo conjunto de obstaculos enviado por robo. Ver _obstaculos_mudaram.
        self._ultimos_obstaculos: dict[int, tuple] = {}

    def run(self):
        status, action = self.root.run()

        # SONDA DA LARGADA (sai com DIAG_JOGO=1) - instrumentacao, nao comportamento.
        #
        # MEDIDO nos replays: os nossos robos so comecam a se mover entre 0,68 e
        # 1,03 s (o goleiro aos 3,73 s), enquanto os amarelos partem aos 0,08 s.
        # O adversario toca a bola aos 0,6-0,7 s e chuta: a partida esta decidida
        # antes de o time sair do lugar.
        #
        # A pergunta que esta sonda responde, e que NAO da para responder por
        # inferencia: a estrategia demorou a MANDAR, ou a cadeia demorou a
        # EXECUTAR? Ela carimba o instante de cada ciclo, o status devolvido pela
        # raiz, quantos comandos sairam, e o instante da primeira publicacao.
        #
        # So os primeiros ciclos interessam - depois disso o log vira ruido.
        #
        # Vai atras do DIAG_JOGO em vez de uma variavel propria: variavel lida
        # pela ESTRATEGIA precisa ser exportada no 'ros_d' do ararabots.sh, que
        # fica fora de src/strategy/. O DIAG_JOGO ja esta plumbado la.
        # O ZERO DO TEMPO E A LARGADA, nao o inicio do no.
        #
        # A primeira versao desta sonda carimbava a partir do primeiro ciclo do
        # strategyNode, que sobe MUITO antes do comando do arbitro - e mostrava
        # 'comandos=4 desde t=0,001', que e verdade e nao responde nada: sob HALT
        # a jogada Halt tambem devolve 4 comandos. O zero tem de ser o instante
        # em que o arbitro manda comecar.
        if os.environ.get("DIAG_JOGO"):
            from strategy.plays.estado_jogo import EstadoJogo
            _m = EstadoJogo.ultimo()
            _cmd = None
            if _m is not None:
                _cmd = getattr(getattr(_m, "referee", None), "command", None)
            if _cmd in ("FORCE_START", "NORMAL_START"):
                if not hasattr(self, "_t0_largada"):
                    self._t0_largada = time.monotonic()
                    self._ciclo_largada = 0
                self._ciclo_largada += 1
                if self._ciclo_largada <= 25:
                    n = 0
                    if action is not None:
                        n = len(action) if isinstance(action, Iterable) else 1
                    _alvos = ""
                    if action is not None and isinstance(action, Iterable):
                        _alvos = " ".join(
                            "r%d:(%.0f,%.0f)" % (s.robot_id, s.target_x or 0.0,
                                                 s.target_y or 0.0)
                            for s in action if hasattr(s, "robot_id"))
                    print("[LG] ciclo=%d t=%.3f status=%s comandos=%d %s"
                          % (self._ciclo_largada,
                             time.monotonic() - self._t0_largada,
                             getattr(status, "name", status), n, _alvos),
                          flush=True)

        if action is None:
            return

        if isinstance(action, Iterable) and not isinstance(action, Skill):
            skill_list = [s for s in action if hasattr(s, "robot_id")]
        else:
            skill_list = [action] if hasattr(action, "robot_id") else []

        latest: dict[int, Skill] = {sk.robot_id: sk for sk in skill_list}

        # No caminho NOVO, junta o ciclo inteiro num array so.
        #
        # O MovementManager faz 'self._movement_commands = msg.commands', ou
        # seja, SUBSTITUI a lista inteira a cada mensagem. Publicando um robo
        # por vez, cada publicacao apaga os alvos dos outros e so o ultimo
        # sobrevive - com o time completo, os demais param.
        self._lote_mov = [] if self.movimento_novo else None

        for sk in latest.values():
            self._send_kick(sk)
            if sk.target_x is not None and sk.target_y is not None:
                # No caminho novo NAO filtramos por "o alvo mudou": o manager so
                # publica alvo para os robos presentes na ultima mensagem, entao
                # omitir um robo estavel e o mesmo que manda-lo parar.
                if self.movimento_novo or self._alvo_mudou(sk):
                    self._send_move(sk)
            if sk.angle is not None:
                self._send_orientation(sk.robot_id, sk.angle)
            if any(
                field
                for field in [
                    sk.field_border,
                    sk.penalty_area,
                    sk.center_area,
                    sk.ball,
                    sk.enemy_ids,
                    sk.ally_ids,
                ]
            ) and self._obstaculos_mudaram(sk):
                self._send_obstacles(sk)

        # NAO SEGURE A PUBLICACAO PARA "EVITAR REPLANEJAMENTO". TESTADO, PIOROU.
        #
        # A hipotese era: publicamos a 10 Hz sem filtro, entao o planejador
        # recebe alvo novo 10 vezes por segundo e reinicia o perfil de
        # trajetoria - dai a rampa lenta medida na largada.
        #
        # MEDIDO, segurando a publicacao enquanto nenhum alvo andava mais que
        # 50 mm (array sempre completo, reenvio de seguranca a cada 0,5 s):
        #
        #   control_reference       10 Hz        segurando
        #     0,5-0,6 s             301 mm/s     85
        #     0,7-0,8 s             551          317
        #     0,9-1,0 s             767          102
        #   velocidade real 1,1-1,2 323          83
        #
        # Piorou tudo, e a referencia passou a OSCILAR (397 -> 175 -> 253) em vez
        # de subir: ela DECAI quando nao e renovada. A cadeia e construida em
        # torno da republicacao continua - o MovementManager substitui a lista
        # inteira a cada mensagem e so publica alvo para quem esta na ultima.
        #
        # De quebra, o portao de medicao do ararabots.sh barra o lote:
        # "/movement_manager/commands a 1 Hz (minimo 8)". Ele esta certo - nao ha
        # como distinguir "quieta de proposito" de "morrendo".
        # TESTE A/B: UM ROBO SO COM MOVIMENTO, OUTRO COM A ESTRATEGIA.
        # (bandeira /tmp/ararabots_ab)
        #
        # A PERGUNTA, do Felipe: a lentidao e da movimentacao ou da estrategia?
        # Se for so da movimentacao, um robo indo de A para B com alvo FIXO deve
        # ter a mesma velocidade e os mesmos erros de um robo comandado pela
        # estrategia. Se o da estrategia for pior, o problema e como NOS mandamos
        # o alvo - que muda a cada ciclo conforme a situacao oscila.
        #
        # POR QUE AQUI, e nao num publicador separado: o MovementManager faz
        # 'self._movement_commands = msg.commands', ou seja, SUBSTITUI a lista a
        # cada mensagem. Dois publicadores no mesmo topico se apagariam um ao
        # outro e o teste mediria a briga, nao a diferenca. Saindo pelo mesmo
        # array, os dois robos recebem comando no mesmo ciclo, pelo mesmo caminho.
        #
        # O robo 3 passa a fazer vaivem entre dois pontos fixos; os demais seguem
        # com a estrategia normal. O alvo dele so muda quando ele CHEGA - que e
        # o oposto do alvo da estrategia, que e reescrito todo ciclo.
        if self.movimento_novo and self._lote_mov and os.path.exists("/tmp/ararabots_ab"):
            # O COMPRIMENTO DO TRAJETO E PARAMETRO, e e o ponto do teste.
            #
            # A primeira rodada usou 6 m e o robo de alvo fixo ficou 45% mais
            # rapido que os da estrategia. So que ele tinha 6 metros para
            # acelerar e os outros perseguem alvos perto da bola - a diferenca
            # podia ser so isso. Escrevendo um numero no arquivo-bandeira, o
            # vaivem passa a ter esse comprimento em mm: com um trajeto CURTO,
            # comparavel ao da estrategia, a comparacao fica honesta.
            try:
                _L = float(open("/tmp/ararabots_ab").read().strip() or 6000.0)
            except Exception:
                _L = 6000.0
            _A, _B = (-_L / 2.0, 2000.0), (_L / 2.0, 2000.0)
            _pos = getattr(self, "_ab_alvo", _B)
            for _c in self._lote_mov:
                if _c.robot_id == 3:
                    _r = None
                    _m = None
                    try:
                        from strategy.plays.estado_jogo import EstadoJogo
                        _m = EstadoJogo.ultimo()
                    except Exception:
                        pass
                    if _m is not None:
                        for _rb in _m.ally_robots:
                            if _rb.id == 3:
                                _r = _rb
                    if _r is not None:
                        _d = ((_r.position_x - _pos[0]) ** 2
                              + (_r.position_y - _pos[1]) ** 2) ** 0.5
                        if _d < 300.0:                    # chegou: inverte
                            _pos = _A if _pos == _B else _B
                            self._ab_alvo = _pos
                    self._ab_alvo = _pos
                    _c.target_pos.x, _c.target_pos.y = _pos[0], _pos[1]
                    if os.environ.get("DIAG_JOGO"):
                        print("[AB] r3 alvo fixo=(%.0f,%.0f)" % _pos, flush=True)

        # UMA publicacao por ciclo, com o time inteiro. Ver _send_move.
        if self.movimento_novo and self._lote_mov:
            msg = self._MovementCommandArray()
            msg.commands = self._lote_mov
            self.mov_pub.publish(msg)
            if os.environ.get("DIAG_JOGO") and not hasattr(self, "_1a_pub"):
                self._1a_pub = time.monotonic()
                print("[LG] PRIMEIRA PUBLICACAO t=%.3f robos=%s"
                      % (self._1a_pub - getattr(self, "_t0_largada", self._1a_pub),
                         [c.robot_id for c in self._lote_mov]), flush=True)

    # Quanto o alvo precisa andar para valer um novo pedido de replanejamento.
    # Abaixo disso e ruido de visao, nao intencao nova.
    LIMIAR_ALVO_MM = 30.0

    # Mesmo com o alvo parado, reenviamos de tempos em tempos.
    #
    # POR QUE: filtrar SO por distancia congelava o robo. Com a bola parada o
    # alvo nao muda, entao apos o primeiro envio nada mais era mandado; a
    # trajetoria terminava onde terminasse e o robo ficava ali. No freekick ele
    # parava a 180 mm da bola - com o chute armado - e nunca encostava, porque o
    # grSim so dispara o chutador em contato real (robot.cpp:160).
    #
    # Reenviar a cada 1,5 s faz o driver replanejar a partir do estado atual e o
    # robo continuar avancando, sem voltar ao problema original de replanejar
    # 10x por segundo (que zerava o time_offset e o fazia se arrastar).
    INTERVALO_REENVIO_S = 1.5

    def _obstaculos_mudaram(self, skill: Skill) -> bool:
        """Este robo precisa mesmo de um novo 'update_obstacles'?

        POR QUE ISTO EXISTE - medicao
        -----------------------------
        Antes mandavamos o conjunto de obstaculos a cada ciclo, para cada robo.
        Com 4 robos sao 40 chamadas por segundo, e cada uma faz o driver
        RECONSTRUIR todos os obstaculos (obstacle_factory.create_obstacles).
        Medimos o efeito: /control_command caiu de 20-40 Hz com um robo para
        3,2 Hz com quatro, e cada robo andou 300 mm em 25 segundos - a jogada
        inteira parou de acontecer.

        O conjunto de obstaculos muda pouco: as flags sao fixas na jogada e as
        listas de vizinhos so mudam quando alguem entra ou sai do raio. Enviar
        so na mudanca tira quase toda essa carga do driver.
        """
        rid = int(skill.robot_id)
        atual = (
            bool(skill.field_border), bool(skill.penalty_area),
            bool(skill.center_area), bool(skill.ball),
            tuple(sorted(skill.enemy_ids or [])),
            tuple(sorted(skill.ally_ids or [])),
        )
        if self._ultimos_obstaculos.get(rid) == atual:
            return False
        self._ultimos_obstaculos[rid] = atual
        return True

    def _alvo_mudou(self, skill: Skill) -> bool:
        """O alvo deste robo mudou o bastante para pedir replanejamento?

        POR QUE ISSO EXISTE
        -------------------
        driver.py:269 zera o time_offset a cada replan - ou seja, TODA chamada a
        'strategy_command' reinicia a trajetoria do zero. Como este node roda a
        10 Hz e reenviava o alvo em todo ciclo, o robo executava eternamente so
        os primeiros 100 ms de uma trajetoria: nunca acelerava e se arrastava a
        poucos milimetros por segundo.

        E o alvo mudava em todo ciclo mesmo com a bola parada, porque a visao tem
        ruido de ~1 mm e as taticas calculam a posicao a partir dela.

        Com o filtro, um alvo estatico e enviado UMA vez; a trajetoria roda
        inteira e o robo chega. Tambem alivia bastante a carga de servicos, que
        vinha estourando o tempo de resposta do driver
        ("failed to send response (timeout)").
        """
        import time as _t

        rid = int(skill.robot_id)
        alvo = (float(skill.target_x), float(skill.target_y))
        agora = _t.time()
        anterior = self._ultimo_alvo.get(rid)
        if anterior is not None:
            dx = alvo[0] - anterior[0]
            dy = alvo[1] - anterior[1]
            perto = (dx * dx + dy * dy) ** 0.5 < self.LIMIAR_ALVO_MM
            recente = (agora - self._ultimo_envio.get(rid, 0.0)) < self.INTERVALO_REENVIO_S
            if perto and recente:
                return False
        self._ultimo_alvo[rid] = alvo
        self._ultimo_envio[rid] = agora
        return True

    def _configurar_manager(self):
        """O MovementManager so publica alvos DEPOIS destes dois servicos.

        ARMADILHA REAL: _ready_to_publish() exige _static_obstacles E
        _goal_keeper_id preenchidos. Sem estas duas chamadas o manager recebe os
        comandos, nao publica alvo nenhum, e NADA se move - sem erro, sem log.
        """
        if self._configurou_manager:
            return
        req = self._SetStaticObstacles.Request()
        req.border_area = True
        req.center_area = False
        self.estaticos_cli.call_async(req)
        req2 = self._SetGoalKeeper.Request()
        req2.robot_id = 0            # o robo 0 e sempre o goleiro nesta base
        self.goleiro_cli.call_async(req2)
        self._configurou_manager = True

    def _send_move(self, skill: Skill) -> None:
        if self.movimento_novo:
            self._configurar_manager()
            cmd = self._MovementCommand()
            cmd.robot_id = int(skill.robot_id)
            cmd.target_pos.x = float(skill.target_x or 0.0)
            cmd.target_pos.y = float(skill.target_y or 0.0)
            if self._lote_mov is None:
                self._lote_mov = []
            self._lote_mov.append(cmd)
            return
        return   # caminho antigo (driver) removido pela dev
    def _send_kick(self, skill: Skill) -> None:
        req = UpdateKick.Request()
        req.id = int(skill.robot_id)
        req.kick = float(skill.kick)
        fut = self.kick_cli.call_async(req)
        fut.add_done_callback(lambda f, rid=req.id: self._handle_kick_response(f, rid))

    def _send_orientation(self, robot_id: int, angle: float) -> None:
        try:
            req = SetOrientation.Request()
        except Exception:
            return
        req.robot_id = int(robot_id)
        req.orientation = float(angle)
        fut = self.orientation_cli.call_async(req)
        fut.add_done_callback(
            lambda f, rid=robot_id: self._handle_orientation_response(f, rid)
        )

    def _send_obstacles(self, skill: Skill) -> None:
        # No caminho NOVO nao existe obstaculo por robo: o MovementManager so
        # aceita border_area/center_area globais (SetStaticObstacles) e deriva o
        # resto do game_state. Os campos por robo que a tatica usa - sobretudo
        # 'ball' - nao tem equivalente, entao nao ha o que enviar aqui.
        #
        # ISTO E UMA PERDA DE COMPORTAMENTO, e esta medida no HANDOVER: a fase
        # de contorno depende da bola ser obstaculo para nao passar por cima
        # dela. Anotado para a comparacao; nao invento equivalente.
        if self.movimento_novo:
            return

        return   # caminho antigo (driver) removido pela dev
    def _handle_kick_response(self, future, robot_id: int) -> None:
        try:
            future.result()
        except Exception as e:
            self.get_logger().error(f"Kick service failed for robot {robot_id}: {e}")

    def _handle_move_response(self, future, robot_id: int) -> None:
        try:
            future.result()
        except Exception as e:
            self.get_logger().error(f"Move service failed for robot {robot_id}: {e}")

    def _handle_orientation_response(self, future, robot_id: int) -> None:
        try:
            future.result()
        except Exception as e:
            self.get_logger().error(
                f"Orientation service failed for robot {robot_id}: {e}"
            )

    def _handle_obstacle_response(self, future, robot_id: int, skill: Skill) -> None:
        try:
            future.result()
        except Exception as e:
            self.get_logger().error(
                f"Obstacle service failed for robot {robot_id}: {e}"
            )


def _traverse_tree(node):
    nodes = [node]
    for child in getattr(node, "children", []):
        nodes.extend(_traverse_tree(child))
    return nodes


def main(args=None):
    rclpy.init(args=args)
    strategy_node = Strategy(wait_for_service=True)

    executor = MultiThreadedExecutor()
    executor.add_node(strategy_node)

    bt_nodes = _traverse_tree(strategy_node.root)
    for n in bt_nodes:
        if isinstance(n, Node):
            executor.add_node(n)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for n in bt_nodes:
            if isinstance(n, Node):
                executor.remove_node(n)
                n.destroy_node()

        executor.remove_node(strategy_node)
        strategy_node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == "__main__":
    main()
