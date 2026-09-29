from system_interfaces.msg._game_state import GameState
from strategy.plays.estado_jogo import EstadoJogo
from strategy.behaviour import LeafNode, Selector, Sequence, TaskStatus
from system_interfaces.srv import GetGameConfig
from strategy.tatics.kickoff import OurKickoff, TheirKickoff


class CheckState(LeafNode):
    def __init__(self, name, _desired_states):
        super().__init__(name)
        self.desired_states = _desired_states
        self.referee_command = None
        EstadoJogo.registrar(self, self.game_state_callback)

    def game_state_callback(self, msg: GameState):
        self.referee_command = msg.referee.command

    def run(self):
        if self.referee_command is None:
            # SEM COMANDO DO ARBITRO -> FAILURE, NUNCA RUNNING.
            #
            # BUG QUE ISTO CORRIGE, e era o maior de todos. O RootTree e um
            # Selector, e Selector para no primeiro filho que nao devolve
            # FAILURE - RUNNING inclusive. O Kickoff vem ANTES do NormalStart.
            # Entao, sempre que o 'game_state' falhava, este RUNNING bloqueava
            # a arvore e o NormalStart nunca rodava: o time inteiro ficava
            # imovel, sem uma linha de log dizendo por que.
            #
            # Medido no replay: nossos robos deslocaram 2, 4 e 4 mm em 24,5 s,
            # enquanto o adversario - comandado direto no grSim pela ferramenta,
            # sem passar pela arvore - andava 1500 mm. A cadeia de movimento
            # estava boa o tempo todo ('mov-bruto': 3448 mm -> 3 mm, CHEGOU).
            # Era a arvore que nunca chegava a pedir nada.
            #
            # Nao saber o comando nao e razao para impedir TODAS as jogadas
            # seguintes. FAILURE deixa o Selector seguir; se for mesmo kickoff,
            # o proximo ciclo (58 Hz) corrige.
            return TaskStatus.FAILURE, None
        return (TaskStatus.SUCCESS, None) if self.referee_command in self.desired_states else (TaskStatus.FAILURE, None)


class CheckIfOurKickoff(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.is_team_color_yellow = None
        self.referee_command = None
        EstadoJogo.registrar(self, self.game_state_callback)
        self.game_config_client = self.create_client(
            GetGameConfig, "get_game_config")
        self._get_color_future = None
        self._config_timer = self.create_timer(0.5, self._request_color_once)

    def game_state_callback(self, msg: GameState):
        self.referee_command = msg.referee.command

    def _request_color_once(self):
        if (
            self.is_team_color_yellow is not None
            or not self.game_config_client.service_is_ready()
            or self._get_color_future is not None
        ):
            return
        req = GetGameConfig.Request()
        self._get_color_future = self.game_config_client.call_async(req)
        self._get_color_future.add_done_callback(self._on_get_color_response)

    def _on_get_color_response(self, future):
        exc = future.exception()
        if exc:
            self.get_logger().warn(f"GetGameConfig failed: {exc}")
        else:
            resp = future.result()
            self.is_team_color_yellow = resp.is_team_color_yellow
        self._get_color_future = None
        if self._config_timer:
            self._config_timer.cancel()
            self._config_timer = None

    def run(self):

        expected_cmd = "PREPARE_KICKOFF_YELLOW" if self.is_team_color_yellow else "PREPARE_KICKOFF_BLUE"

        if self.referee_command == expected_cmd:
            return TaskStatus.SUCCESS, None
        return TaskStatus.FAILURE, None


class OurKickoffAction(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.ally_robots = {}
        self.on_positive_half = None
        EstadoJogo.registrar(self, self.game_state_callback)
        self.game_config_client = self.create_client(
            GetGameConfig, "get_game_config")
        self._get_color_future = None
        self._config_timer = self.create_timer(0.5, self._request_color_once)

    def _request_color_once(self):
        if (
            self.on_positive_half is not None
            or not self.game_config_client.service_is_ready()
            or self._get_color_future is not None
        ):
            return
        req = GetGameConfig.Request()
        self._get_color_future = self.game_config_client.call_async(req)
        self._get_color_future.add_done_callback(self._on_get_color_response)

    def _on_get_color_response(self, future):
        exc = future.exception()
        if exc:
            self.get_logger().warn(f"GetGameConfig failed: {exc}")
        else:
            resp = future.result()
            self.on_positive_half = resp.on_positive_half
        self._get_color_future = None
        if self._config_timer:
            self._config_timer.cancel()
            self._config_timer = None

    def game_state_callback(self, msg: GameState):
        self.ally_robots = {r.id: r for r in msg.ally_robots}

    def run(self):

        if not self.ally_robots or self.on_positive_half is None:
            return TaskStatus.RUNNING, None

        executor = OurKickoff(ally_robots=self.ally_robots,
                              on_positive_half=self.on_positive_half)

        return TaskStatus.SUCCESS, executor.execute()


class TheirKickoffAction(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.ally_robots = {}
        self.on_positive_half = None
        EstadoJogo.registrar(self, self.game_state_callback)
        self.game_config_client = self.create_client(
            GetGameConfig, "get_game_config")
        self._get_color_future = None
        self._config_timer = self.create_timer(0.5, self._request_color_once)

    def _request_color_once(self):
        if (
            self.on_positive_half is not None
            or not self.game_config_client.service_is_ready()
            or self._get_color_future is not None
        ):
            return
        req = GetGameConfig.Request()
        self._get_color_future = self.game_config_client.call_async(req)
        self._get_color_future.add_done_callback(self._on_get_color_response)

    def _on_get_color_response(self, future):
        exc = future.exception()
        if exc:
            self.get_logger().warn(f"GetGameConfig failed: {exc}")
        else:
            resp = future.result()
            self.on_positive_half = resp.on_positive_half
        self._get_color_future = None
        if self._config_timer:
            self._config_timer.cancel()
            self._config_timer = None

    def game_state_callback(self, msg: GameState):
        self.ally_robots = {r.id: r for r in msg.ally_robots}

    def run(self):

        if not self.ally_robots or self.on_positive_half is None:
            return TaskStatus.RUNNING, None

        executor = TheirKickoffAction(
            ally_robots=self.ally_robots, on_positive_half=self.on_positive_half)

        return TaskStatus.SUCCESS, executor.execute()


class Kickoff(Sequence):
    def __init__(self, name):
        super().__init__(name, [])

        """ List with possible inputs to this state """

        commands = ["PREPARE_KICKOFF_BLUE", "PREPARE_KICKOFF_YELLOW"]

        check_kickoff = CheckState("CheckKickoff", commands)

        is_ours = CheckIfOurKickoff("CheckIfOurKickoff")
        action_ours = OurKickoffAction("OurKickoffAction")

        ours = Sequence("OurKickoff", [is_ours, action_ours])

        action_theirs = TheirKickoffAction("TheirKickoffAction")

        ours_or_theirs = Selector("OursOrTheirsKickoff", [ours, action_theirs])

        self.add_children([check_kickoff, ours_or_theirs])

    def run(self):
        """Access the second element in tuple"""
        return super().run()
