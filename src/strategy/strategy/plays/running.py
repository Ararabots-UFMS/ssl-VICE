from new_movement.entities.States import Vector2D
from strategy.behaviour import Selector, Sequence, LeafNode, TaskStatus
from system_interfaces.msg._game_state import GameState
from system_interfaces.srv import GetGameConfig
from strategy.tatics.running import Atack, Defense


class CheckState(LeafNode):
    def __init__(self, name, desired_states):
        super().__init__(name)
        self.desired_states = desired_states
        self.referee_command = None
        self.create_subscription(GameState, "game_state", self.game_state_callback, 10)

    def game_state_callback(self, msg: GameState):
        self.referee_command = msg.referee.command

    def run(self):
        return (
            (TaskStatus.SUCCESS, None)
            if self.referee_command in self.desired_states
            else (TaskStatus.FAILURE, None)
        )


class CheckAtack(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.ball = None
        self.on_positive_half = None
        self.create_subscription(GameState, "game_state", self.game_state_callback, 10)
        self.game_config_client = self.create_client(GetGameConfig, "get_game_config")
        self._get_half_future = None
        self._config_timer = self.create_timer(0.5, self._request_half_once)

    def game_state_callback(self, msg: GameState):
        self.ally_robots = {r.id: r for r in msg.ally_robots}
        try:
            self.ball = msg.balls[0]
        except IndexError:
            self.ball = None

    def _request_half_once(self):
        if (
            self.on_positive_half is not None
            or not self.game_config_client.service_is_ready()
            or self._get_half_future is not None
        ):
            return
        req = GetGameConfig.Request()
        self._get_half_future = self.game_config_client.call_async(req)
        self._get_half_future.add_done_callback(self._on_get_half_response)

    def _on_get_half_response(self, future):
        exc = future.exception()
        if exc:
            self.get_logger().warn(f"GetGameConfig failed: {exc}")
        else:
            resp = future.result()
            self.on_positive_half = resp.on_positive_half
        self._get_half_future = None
        if self._config_timer:
            self._config_timer.cancel()
            self._config_timer = None

    def _get_border_circle(self) -> Vector2D:
        radius = 500
        if self.on_positive_half:
            return Vector2D(radius, 0.0)
        return Vector2D(-radius, 0.0)

    def run(self):
        if self.ball is None or self.on_positive_half is None:
            return TaskStatus.RUNNING, None

        border_circle = self._get_border_circle()

        if self.on_positive_half:
            if self.ball.position_x < border_circle.x:
                return TaskStatus.SUCCESS, None
        else:
            if self.ball.position_x > border_circle.x:
                return TaskStatus.SUCCESS, None

        return TaskStatus.FAILURE, None


class AtackAction(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.ally_robots = None
        self.enemy_robots = None
        self.on_positive_half = None
        self.create_subscription(GameState, "game_state", self.game_state_callback, 10)
        self.game_config_client = self.create_client(GetGameConfig, "get_game_config")
        self._config_timer = self.create_timer(0.5, self._request_half_once)
        self._get_half_future = None

    def game_state_callback(self, msg: GameState):
        self.ally_robots = {r.id: r for r in msg.ally_robots}
        self.enemy_robots = {r.id: r for r in msg.enemy_robots}

        try:
            self.ball = msg.balls[0]
        except IndexError:
            self.ball = None

    def _request_half_once(self):
        if (
            self.on_positive_half is not None
            or not self.game_config_client.service_is_ready()
            or self._get_half_future is not None
        ):
            return
        req = GetGameConfig.Request()
        self._get_half_future = self.game_config_client.call_async(req)
        self._get_half_future.add_done_callback(self._on_get_half_response)

    def _on_get_half_response(self, future):
        exc = future.exception()
        if exc:
            self.get_logger().warn(f"GetGameConfig failed: {exc}")
        else:
            resp = future.result()
            self.on_positive_half = resp.on_positive_half
        self._get_half_future = None
        if self._config_timer:
            self._config_timer.cancel()
            self._config_timer = None

    def run(self):
        if (
            self.ally_robots is None
            and self.ball is None
            and self.on_positive_half is None
        ):
            return TaskStatus.RUNNING, None

        atacker = Atack(
            ally_robots=self.ally_robots,
            enemy_robots=self.enemy_robots,
            ball=self.ball,
            on_positive_half=self.on_positive_half,
        )

        return TaskStatus.SUCCESS, atacker.execute()


class DefenseAction(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.ally_robots = None
        self.enemy_robots = None
        self.on_positive_half = None
        self.create_subscription(GameState, "game_state", self.game_state_callback, 10)
        self.game_config_client = self.create_client(GetGameConfig, "get_game_config")
        self._config_timer = self.create_timer(0.5, self._request_half_once)
        self._get_half_future = None

    def game_state_callback(self, msg: GameState):
        self.ally_robots = {r.id: r for r in msg.ally_robots}
        self.enemy_robots = {r.id: r for r in msg.enemy_robots}

        try:
            self.ball = msg.balls[0]
        except IndexError:
            self.ball = None

    def _request_half_once(self):
        if (
            self.on_positive_half is not None
            or not self.game_config_client.service_is_ready()
            or self._get_half_future is not None
        ):
            return
        req = GetGameConfig.Request()
        self._get_half_future = self.game_config_client.call_async(req)
        self._get_half_future.add_done_callback(self._on_get_half_response)

    def _on_get_half_response(self, future):
        exc = future.exception()
        if exc:
            self.get_logger().warn(f"GetGameConfig failed: {exc}")
        else:
            resp = future.result()
            self.on_positive_half = resp.on_positive_half
        self._get_half_future = None
        if self._config_timer:
            self._config_timer.cancel()
            self._config_timer = None

    def run(self):
        if (
            self.ally_robots is None
            and self.ball is None
            and self.on_positive_half is None
        ):
            return TaskStatus.RUNNING, None

        atacker = Atack(
            ally_robots=self.ally_robots,
            enemy_robots=self.enemy_robots,
            ball=self.ball,
            on_positive_half=self.on_positive_half,
        )

        return TaskStatus.SUCCESS, atacker.execute()


class NormalStart(Sequence):
    def __init__(self, name):
        super().__init__(name, [])

        commands = ["FORCE_START", "NORMAL_START"]

        can_i_start = CheckState("CheckState", commands)

        i_can_atack = CheckAtack("CheckAtack")

        atack_action = AtackAction("AtackAction")

        defense_action = DefenseAction("DefenseAction")

        can_i_atack = Sequence("CanIAttack", [i_can_atack, atack_action])

        start = Selector("Start", [can_i_atack, defense_action])

        self.add_children([can_i_start, start])

    def run(self):
        return super().run()
