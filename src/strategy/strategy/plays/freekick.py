from system_interfaces.msg._game_state import GameState
from strategy.behaviour import LeafNode, Selector, Sequence, TaskStatus
from system_interfaces.srv import GetGameConfig
import time

from strategy.tatics.freekick import (
    OurFreekick,
    TheirFreekick,
    EstadoFreekick,
    MedidasCampo,
)


class CheckState(LeafNode):
    def __init__(self, name, _desired_states):
        super().__init__(name)
        self.desired_states = _desired_states
        self.referee_command = None
        self.create_subscription(GameState, "game_state", self.game_state_callback, 10)

    def game_state_callback(self, msg: GameState):
        self.referee_command = msg.referee.command

    def run(self):
        return (TaskStatus.SUCCESS, None) if self.referee_command in self.desired_states else (TaskStatus.FAILURE, None)


class CheckIfOurFreekick(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.is_team_color_yellow = None
        self.referee_command = None
        self.create_subscription(GameState, "game_state", self.game_state_callback, 10)
        self.game_config_client = self.create_client(GetGameConfig, "get_game_config")
        self._get_color_future = None
        self._pedido_em = 0.0
        self._config_timer = self.create_timer(0.5, self._request_color_once)

    def game_state_callback(self, msg: GameState):
        self.referee_command = msg.referee.command

    def _request_color_once(self):
        # Se a chamada anterior nunca respondeu, tenta de novo.
        #
        # Antes o guard era so '_get_color_future is not None': bastava a
        # primeira chamada ficar pendurada (por exemplo, se o strategyNode subiu
        # antes do gameWatcher) para o node nunca mais tentar. Ele ficava preso
        # em RUNNING, e como o Selector devolve no primeiro filho que NAO falha,
        # a jogada inteira parava sem gerar comando nenhum.
        if self.is_team_color_yellow is not None:
            return
        if self._get_color_future is not None:
            if time.time() - self._pedido_em < 3.0:
                return
            self._get_color_future = None      # expirou: libera nova tentativa
        if not self.game_config_client.service_is_ready():
            return
        req = GetGameConfig.Request()
        self._pedido_em = time.time()
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

        if self.is_team_color_yellow is None:
            return TaskStatus.RUNNING, None

        expected_cmd = "DIRECT_FREE_YELLOW" if self.is_team_color_yellow else "DIRECT_FREE_BLUE"

        if self.referee_command == expected_cmd:
            return TaskStatus.SUCCESS, None
        return TaskStatus.FAILURE, None


class _AcaoFreekick(LeafNode):
    """Parte comum as duas acoes: dados do jogo, medidas e estado da jogada.

    O estado (quem chutou por ultimo, quando a jogada comecou) e RECEBIDO da
    jogada Freekick. Antes cada acao tentava ler isso de um 'parent' da arvore,
    que nao existe em behaviour.py: last_kicker_id era sempre None, a protecao de
    duplo toque nunca valia e o cronometro nunca contava.
    """

    def __init__(self, name, estado: EstadoFreekick):
        super().__init__(name)
        self.estado = estado
        self.ally_robots = {}
        self.enemy_robots = {}
        self.balls = []
        self.medidas = MedidasCampo()          # padrao Division B
        self.on_positive_half = None
        self.create_subscription(GameState, "game_state", self.game_state_callback, 10)
        self.game_config_client = self.create_client(GetGameConfig, "get_game_config")
        self._get_color_future = None
        self._pedido_em = 0.0
        self._config_timer = self.create_timer(0.5, self._request_color_once)

    def _request_color_once(self):
        # Se a chamada anterior nunca respondeu, tenta de novo.
        #
        # Antes o guard era so '_get_color_future is not None': bastava a
        # primeira chamada ficar pendurada (por exemplo, se o strategyNode subiu
        # antes do gameWatcher) para o node nunca mais tentar. Ele ficava preso
        # em RUNNING, e como o Selector devolve no primeiro filho que NAO falha,
        # a jogada inteira parava sem gerar comando nenhum.
        if self.on_positive_half is not None:
            return
        if self._get_color_future is not None:
            if time.time() - self._pedido_em < 3.0:
                return
            self._get_color_future = None      # expirou: libera nova tentativa
        if not self.game_config_client.service_is_ready():
            return
        req = GetGameConfig.Request()
        self._pedido_em = time.time()
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
        # Adversarios passam a ser usados como obstaculos pelo planejador.
        self.enemy_robots = {r.id: r for r in msg.enemy_robots}
        self.balls = msg.balls
        # As medidas vem da visao, em vez de numeros fixos no codigo.
        self.medidas = MedidasCampo.da_geometria(msg.geometry)

    def _pronto(self) -> bool:
        return bool(self.ally_robots) and self.balls and self.on_positive_half is not None

    def _executar(self, classe_tatica):
        executor = classe_tatica(
            ally_robots=self.ally_robots,
            ball=self.balls[0],
            on_positive_half=self.on_positive_half,
            enemy_robots=self.enemy_robots,
            estado=self.estado,
            medidas=self.medidas,
        )
        status, commands = executor.execute()
        return status, commands


class OurFreekickAction(_AcaoFreekick):
    def run(self):
        if not self._pronto():
            return TaskStatus.RUNNING, None
        return self._executar(OurFreekick)


class TheirFreekickAction(_AcaoFreekick):
    def run(self):
        if not self._pronto():
            return TaskStatus.RUNNING, None
        return self._executar(TheirFreekick)


class Freekick(Sequence):
    """Jogada de falta. Dona do estado que as acoes compartilham."""

    def __init__(self, name):
        super().__init__(name, [])

        # Uma unica instancia, injetada nas duas acoes: e assim que o
        # last_kicker_id sobrevive de um ciclo para o outro.
        self.estado = EstadoFreekick()
        self._ultimo_comando = None

        """ List with possible inputs to this state """

        commands = ["DIRECT_FREE_BLUE", "DIRECT_FREE_YELLOW"]

        self.check_freekick = CheckState("CheckFreekick", commands)

        is_ours = CheckIfOurFreekick("CheckIfOurFreekick")
        action_ours = OurFreekickAction("OurFreekickAction", self.estado)

        ours = Sequence("OurFreekick", [is_ours, action_ours])

        action_theirs = TheirFreekickAction("TheirFreekickAction", self.estado)

        ours_or_theirs = Selector("OursOrTheirsFreeKick", [ours, action_theirs])

        self.add_children([self.check_freekick, ours_or_theirs])

    def run(self):
        """Access the second element in tuple"""
        # Cada nova falta e uma jogada nova: zera cronometro e duplo toque.
        # Sem isso o limite de 10 s de uma cobranca vazaria para a seguinte.
        comando = self.check_freekick.referee_command
        if comando != self._ultimo_comando:
            self.estado.reiniciar()
            self._ultimo_comando = comando
        return super().run()
