from system_interfaces.msg._game_state import GameState
from strategy.behaviour import LeafNode, Selector, Sequence, TaskStatus
from system_interfaces.srv import GetGameConfig
from strategy.tatics.freekick import OurFreekick


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

        expected_cmd = "DIRECT_FREE_YELLOW" if self.is_team_color_yellow else "DIRECT_FREE_BLUE"

        if self.referee_command == expected_cmd:
            return TaskStatus.SUCCESS, None
        return TaskStatus.FAILURE, None


class OurFreekickAction(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.ally_robots = {}
        self.on_positive_half = None
        self.create_subscription(GameState, "game_state", self.game_state_callback, 10)
        self.game_config_client = self.create_client(GetGameConfig, "get_game_config")
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

        executor = OurKickoff(ally_robots=self.ally_robots, on_positive_half=self.on_positive_half)

        return TaskStatus.SUCCESS, executor.execute()

class TheirFreekickAction(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.ally_robots = {}
        self.on_positive_half = None
        self.create_subscription(GameState, "game_state", self.game_state_callback, 10)
        self.game_config_client = self.create_client(GetGameConfig, "get_game_config")
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

        executor = OurFreekick(ally_robots=self.ally_robots, on_positive_half=self.on_positive_half)

        return TaskStatus.SUCCESS, executor.execute()


class Freekick(Sequence):
    def __init__(self, name):
        super().__init__(name, [])

        """ List with possible inputs to this state """

        commands = ["DIRECT_FREE_BLUE", "DIRECT_FREE_YELLOW"]

        check_freekick = CheckState("CheckFreekick", commands)

        is_ours = CheckIfOurFreekick("CheckIfOurFreekick")
        action_ours = OurFreekickAction("OurFreekickAction")

        ours = Sequence("OurFreekick", [is_ours, action_ours])

        action_theirs = TheirFreekickAction("TheirFreekickAction")

        ours_or_theirs = Selector("OursOrTheirsFreeKick", [ours, action_theirs])

        self.add_children([check_kickoff, ours_or_theirs])


    def run(self):
        """Access the second element in tuple"""
        return super().run()
