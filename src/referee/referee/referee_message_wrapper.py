from system_interfaces.msg import RefereeMessage, TeamData
from referee.proto.ssl_gc_referee_message_pb2 import Referee

class MessageWrapping():
    """It is responsible to filter the data send by the game-controller and populate our Data"""
    def __init__(self, referee_message):
        self.msg = RefereeMessage()
        blue_team = TeamData()
        yellow_team = TeamData()

        self.msg.teams.append(blue_team)
        self.msg.teams.append(yellow_team)
        
        self.referee = referee_message
    
    def to_game_data(self):
        self.msg.stage = Referee.Stage.Name(self.referee.stage)
        self.msg.command = Referee.Command.Name(self.referee.command)
        self.msg.command_counter = self.referee.command_counter

    def blue_team_description(self):
        self.msg.teams[0].color = 'blue'
        self.msg.teams[0].name = self.referee.blue.name
        self.msg.teams[0].score = self.referee.blue.score
        self.msg.teams[0].timeouts = self.referee.blue.timeouts
        self.msg.teams[0].goalkeeper = self.referee.blue.goalkeeper
        self.msg.teams[0].foul_counter = self.referee.blue.foul_counter
        self.msg.teams[0].ball_placement_failures = self.referee.blue.ball_placement_failures
        self.msg.teams[0].can_place_ball = self.referee.blue.can_place_ball
        self.msg.teams[0].max_allowed_bots = self.referee.blue.max_allowed_bots
        self.msg.teams[0].bot_substitution_intent = self.referee.blue.bot_substitution_intent
        self.msg.teams[0].bot_substitution_allowed = self.referee.blue.bot_substitution_allowed
        self.msg.teams[0].bot_substitutions_left = self.referee.blue.bot_substitutions_left
        self.msg.teams[0].red_cards = self.referee.blue.red_cards
        self.msg.teams[0].yellow_cards = self.referee.blue.yellow_cards

    def yellow_team_description(self):
        self.msg.teams[1].color = 'yellow'
        self.msg.teams[1].name = self.referee.yellow.name
        self.msg.teams[1].score = self.referee.yellow.score
        self.msg.teams[1].timeouts = self.referee.yellow.timeouts
        self.msg.teams[1].goalkeeper = self.referee.yellow.goalkeeper
        self.msg.teams[1].foul_counter = self.referee.yellow.foul_counter
        self.msg.teams[1].ball_placement_failures = self.referee.yellow.ball_placement_failures
        self.msg.teams[1].can_place_ball = self.referee.yellow.can_place_ball
        self.msg.teams[1].max_allowed_bots = self.referee.yellow.max_allowed_bots
        self.msg.teams[1].bot_substitution_intent = self.referee.yellow.bot_substitution_intent
        self.msg.teams[1].bot_substitution_allowed = self.referee.yellow.bot_substitution_allowed
        self.msg.teams[1].bot_substitutions_left = self.referee.yellow.bot_substitutions_left
        self.msg.teams[1].red_cards = self.referee.yellow.red_cards
        self.msg.teams[1].yellow_cards = self.referee.yellow.yellow_cards
