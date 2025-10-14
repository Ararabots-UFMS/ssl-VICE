from system_interfaces.msg._game_state import GameState
from strategy.skills.skills import Skills
from strategy.behaviour import LeafNode, Selector, Sequence, TaskStatus
from system_interfaces.srv import GetGameConfig
from strategy.tatics.kickoff import OurKickoff


class CheckState(LeafNode):
    def __init__(self, name, _desired_states, referee_command=None):
        super().__init__(name)
        self.desired_states = _desired_states
        self.referee_command = referee_command

    def run(self):
        return (TaskStatus.SUCCESS, None) if self.referee_command in self.desired_states else (TaskStatus.FAILURE, None)


class CheckIfOurKickoff(LeafNode):
    def __init__(self, name, referee_command=None, is_team_color_yellow=None):
        super().__init__(name)
        self.is_team_color_yellow = is_team_color_yellow
        self.referee_command = referee_command

    def run(self):

        expected_cmd = "PREPARE_KICKOFF_YELLOW" if self.is_team_color_yellow else "PREPARE_KICKOFF_BLUE"

        if self.referee_command == expected_cmd:
            return TaskStatus.SUCCESS, None
        return TaskStatus.FAILURE, None


class OurKickoffAction(LeafNode):
    def __init__(self, name, on_positive_half=None, ally_robots=None):
        super().__init__(name)
        self.ally_robots = ally_robots
        self.on_positive_half = on_positive_half

    def run(self):

        if not self.ally_robots or self.on_positive_half is None:
            return TaskStatus.RUNNING, None

        executor = OurKickoff(ally_robots=self.ally_robots, on_positive_half=self.on_positive_half)
        return TaskStatus.SUCCESS, executor.execute()


class TheirKickoffAction(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.commands = {}

    def run(self):

        skills_factory = Skills("Movement")

        r0 = skills_factory.move_to(robot_id=0, target_x=-1000.0, target_y=-1000.0, vel_x=0.0, vel_y=0.0)
        r0.field_border = True

        return TaskStatus.SUCCESS, [r0]


class Kickoff(Sequence):
    def __init__(self, name):
        super().__init__(name, [])

        """ List with possible inputs to this state """

        self.create_subscription(GameState, "game_state", self.game_state_callback, 10)
        self.game_config_client = self.create_client(GetGameConfig, "get_game_config")
        self._get_color_future = None
        self._config_timer = self.create_timer(0.5, self._request_color_once)

        commands = ["PREPARE_KICKOFF_BLUE", "PREPARE_KICKOFF_YELLOW"]

        check_kickoff = CheckState("CheckKickoff", commands, self.referee_command)

        is_ours = CheckIfOurKickoff("CheckIfOurKickoff", self.referee_command)
        action_ours = OurKickoffAction("OurKickoffAction")

        ours = Sequence("OurKickoff", [is_ours, action_ours])

        action_theirs = TheirKickoffAction("TheirKickoffAction")

        ours_or_theirs = Selector("OursOrTheirsKickoff", [ours, action_theirs])

        self.add_children([check_kickoff, ours_or_theirs])

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
        """Access the second element in tuple"""
        return super().run()
