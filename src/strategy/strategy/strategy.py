import os

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from strategy.root import RootTree
from typing import Iterable
from system_interfaces.srv import StrategyCommand, SetOrientation, UpdateObstacle, UpdateKick
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
        self.movimento_novo = bool(os.environ.get("MOVIMENTO_NOVO"))

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
            self.move_cli = self.create_client(StrategyCommand, "strategy_command")
        self.orientation_cli = self.create_client(SetOrientation, "set_orientation")
        self.obstacle_cli = self.create_client(UpdateObstacle, "update_obstacles")
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

        self.timer = self.create_timer(0.1, self.run)
        self.root = RootTree("RootStrategy")

        # Ultimo alvo enviado por robo, para nao repedir a mesma coisa.
        self._ultimo_alvo: dict[int, tuple] = {}
        self._ultimo_envio: dict[int, float] = {}
        # Ultimo conjunto de obstaculos enviado por robo. Ver _obstaculos_mudaram.
        self._ultimos_obstaculos: dict[int, tuple] = {}

    def run(self):
        status, action = self.root.run()

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

        # UMA publicacao por ciclo, com o time inteiro. Ver _send_move.
        if self.movimento_novo and self._lote_mov:
            msg = self._MovementCommandArray()
            msg.commands = self._lote_mov
            self.mov_pub.publish(msg)

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
        req = StrategyCommand.Request()
        req.id = int(skill.robot_id)
        req.position_x = float(skill.target_x or 0.0)
        req.position_y = float(skill.target_y or 0.0)
        req.velocity_x = float(skill.vel_x)
        req.velocity_y = float(skill.vel_y)
        fut = self.move_cli.call_async(req)
        fut.add_done_callback(lambda f, rid=req.id: self._handle_move_response(f, rid))

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

        req = UpdateObstacle.Request()
        req.id = int(skill.robot_id)
        req.field_border = bool(skill.field_border)
        req.penalty_area = bool(skill.penalty_area)
        req.center_area = bool(skill.center_area)
        req.ball = bool(skill.ball)
        req.enemy_ids = list(skill.enemy_ids or [])
        req.ally_ids = list(skill.ally_ids or [])
        fut = self.obstacle_cli.call_async(req)
        fut.add_done_callback(
            lambda f, rid=req.id: self._handle_obstacle_response(f, rid, skill)
        )

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
