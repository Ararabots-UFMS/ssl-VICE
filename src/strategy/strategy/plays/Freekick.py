from strategy.behaviour import Sequence, Selector, LeafNode, TaskStatus
from system_interfaces.msg import GameState
from strategy.plays.Attack_Freekick import AttackFreekick
from strategy.plays.Defense_Freekick import DefenseFreekick
from system_interfaces.srv import GetGameConfig

class Freekick(Sequence):
    def __init__(self, name):
        super().__init__(name, [])
        commands = ["PREPARE_FREE_YELLOW", "PREPARE_FREE_BLUE"]

        check_freekick = CheckFreekickState("CheckFreekickState")

        is_our_freekick = CheckIsOurFreekick("CheckIsOurFreekick", [check_freekick])
        action_our_freekick = OurFreekick("OurFreekick")
        
        ours = Sequence("OurFreekick", [is_our_freekick, action_our_freekick])

        action_theirs = TheirFreekick("TheirFreekick")

        ours_or_theirs = Selector("OursOrTheirs", [ours, action_theirs])

        self.add_children([check_freekick, ours_or_theirs])
    
    def run(self):
        return super().run()
    
class CheckFreekickState(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.game_state_sub = self.create_subscription(GameState, 'game_state', self.game_state_callback, 10)


    def game_state_callback(self, msg: GameState):
        self.referee_command = msg.referee_command

    def run(self):
        return (TaskStatus.SUCCESS, None) if self.referee_command in self.desired_states else (TaskStatus.FAILURE, None)
class CheckIsOurFreekick(Selector):
    def __init__(self, name, children):
        super().__init__(name, children)
        is_team_color_yellow = None
        self.referee_command = None

        self.game_config_client = self.create_client(GetGameConfig, "get_game_config")
        self._get_color_future = None
        self._config_timer = self.create_timer(0.5, self._request_color_once)
        self.create_subscription(GameState, 'game_state', self.game_state_callback, 10)

    def game_state_callback(self, msg: GameState):
        self.referee_command = msg.referee_command

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
            self.get_logger().error(f"Service call failed {exc}")
        else:
            resp = future.result()
            self.is_team_color_yellow = resp.is_team_color_yellow
            self._get_color_future = None
            if self._config_timer:
                self._config_timer.cancel()
                self._config_timer = None

    def run(self):
        expected_cmd = "PREPARE_FREE_YELLOW" if self.is_team_color_yellow else "PREPARE_FREE_BLUE"
        if self.referee_command == expected_cmd:
            return TaskStatus.SUCCESS, None
        return

class OurFreekick(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.commands = {}

    def run(self):
        self.add_children([AttackFreekick("AttackFreekick")])
        return TaskStatus.SUCCESS, None

class TheirFreekick(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.commands = {}

    def run(self):
        self.add_children([DefenseFreekick("DefenseFreekick")])
        return TaskStatus.SUCCESS, None