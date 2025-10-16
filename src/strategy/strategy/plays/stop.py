from system_interfaces.srv import GetGameConfig
from system_interfaces.msg._game_state import GameState
from strategy.skills.skills import Skills
from strategy.behaviour import LeafNode, Selector, Sequence, TaskStatus
from strategy.tatics.stop import goAwayFromBall


import math


class CheckState(LeafNode):
    def __init__(self, name, _desired_states):
        super().__init__(name)
        self.desired_states = _desired_states
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


class CheckDistance(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.ally_robots = None
        self.ball = None
        self.create_subscription(GameState, "game_state", self.game_state_callback, 10)

    def game_state_callback(self, msg: GameState):
        self.ally_robots = msg.ally_robots
        try:
            self.ball = msg.balls[0]
        except Exception:
            self.ball = None

    def distance_to_ball(self):
        for robot in self.ally_robots:
            distance = (
                (robot.position_x - self.ball.position_x) ** 2
                + (robot.position_y - self.ball.position_y) ** 2
            ) ** 0.5
            if distance < 500:  # distância em mm
                return False

        return True

    def run(self):
        if self.ally_robots is None or self.ball is None:
            return TaskStatus.RUNNING, None

        distance = self.distance_to_ball()

        if distance:
            return TaskStatus.SUCCESS, None
        return TaskStatus.FAILURE, None


class KeepPosition(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.ally_robots = None
        self.create_subscription(GameState, "game_state", self.game_state_callback, 10)

    def game_state_callback(self, msg: GameState):
        self.ally_robots = {r.id: r for r in msg.ally_robots}

    def run(self):

        return TaskStatus.SUCCESS, None


class GetoffBall(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.ally_robots = None
        self.ball = None
        self.create_subscription(GameState, "game_state", self.game_state_callback, 10)
        self.game_config_client = self.create_client(GetGameConfig, "get_game_config")
        self.on_positive_half = None
        self._get_half_future = None
        self._config_timer = self.create_timer(0.5, self._request_half_once)

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

    def game_state_callback(self, msg: GameState):
        self.ally_robots = {r.id: r for r in msg.ally_robots}
        try:
            self.ball = msg.balls[0]
        except Exception:
            self.ball = None

    def run(self):
        if self.ally_robots is None or self.ball is None:
            return TaskStatus.RUNNING, None

        go_away = goAwayFromBall(self.ally_robots, self.ball, self.on_positive_half)

        return TaskStatus.SUCCESS, go_away.execute()


class Stop(Sequence):
    def __init__(self, name):
        super().__init__(name, [])
        self.ally_robots = None
        self.ball = None
        self.create_subscription(GameState, "game_state", self.game_state_callback, 10)

        game_state_is_stop = CheckState("game_state_is_stop", ["STOP"])

        robot_is_far_from_ball = CheckDistance("robot_is_far_from_ball")

        maintain_current_position = KeepPosition("maintain_current_position")

        move_away_from_ball = GetoffBall("move_away_from_ball")

        allowed_to_stop = Sequence(
            "allowed_to_stop", [robot_is_far_from_ball, maintain_current_position]
        )

        decide_stop_or_move_away = Selector(
            "decide_stop_or_move_away", [allowed_to_stop, move_away_from_ball]
        )

        self.add_children([game_state_is_stop, decide_stop_or_move_away])

    def game_state_callback(self, msg: GameState):
        self.ally_robots = {r.id: r for r in msg.ally_robots}
        try:
            self.ball = msg.balls[0]
        except Exception:
            self.ball = None

    def run(self):
        """Access the second element in tuple"""
        return super().run()
