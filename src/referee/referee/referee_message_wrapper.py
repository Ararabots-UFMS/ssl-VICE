from system_interfaces.msg import RefereeMessage, TeamData
from referee.proto.ssl_gc_referee_message_pb2 import Referee

BLUE = "blue"
YELLOW = "yellow"


class MessageWrapping:
    """Converts Referee protobuf message into a RefereeMessage ROS message."""

    def __init__(self, referee_message):
        self.referee = referee_message
        self.msg = self.wrap()

    def wrap(self) -> RefereeMessage:
        # Create a new RefereeMessage with the teams
        msg = RefereeMessage()
        msg.stage = Referee.Stage.Name(self.referee.stage)
        msg.stage_time_left = self.referee.stage_time_left
        msg.command = Referee.Command.Name(self.referee.command)
        msg.command_counter = self.referee.command_counter
        msg.current_action_time_remaining = self.referee.current_action_time_remaining

        msg.teams.append(self._build_team(self.referee.blue, BLUE))
        msg.teams.append(self._build_team(self.referee.yellow, YELLOW))

        return msg

    def _build_team(self, team_proto, color: str) -> TeamData:
        team = TeamData()
        team.color = color
        team.name = team_proto.name
        team.score = team_proto.score
        team.red_cards = team_proto.red_cards
        team.yellow_cards = team_proto.yellow_cards
        team.timeouts = team_proto.timeouts
        team.goalkeeper = team_proto.goalkeeper
        team.foul_counter = team_proto.foul_counter
        team.ball_placement_failures = team_proto.ball_placement_failures
        team.can_place_ball = team_proto.can_place_ball
        team.max_allowed_bots = team_proto.max_allowed_bots
        team.bot_substitution_intent = team_proto.bot_substitution_intent
        team.bot_substitution_allowed = team_proto.bot_substitution_allowed
        team.bot_substitutions_left = team_proto.bot_substitutions_left
        blue_on_positive = bool(getattr(self.referee, "blue_team_on_positive_half", False))
        team.on_positive_half = blue_on_positive if color == BLUE else not blue_on_positive

        return team
