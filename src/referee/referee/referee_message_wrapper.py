from system_interfaces.msg import GameData, TeamData
from referee.proto.ssl_gc_referee_message_pb2 import Referee
class MessageWrapping():
    def __init__(self, referee_message):
        self.msg = GameData()
        blue_team = TeamData()
        yellow_team = TeamData()

        self.msg.teams.append(blue_team)
        self.msg.teams.append(yellow_team)
        
        self.referee = referee_message
    
    def to_game_data(self):
        # TODO: create a class to wrapper those messages
        self.msg.stage = Referee.Stage.Name(self.referee.stage)
        self.msg.command = Referee.Command.Name(self.referee.command)
        self.msg.command_counter = self.referee.command_counter

    def blue_team_description(self):
        self.msg.teams[0].color = 'blue'
        self.msg.teams[0].score = self.referee.blue.score
        self.msg.teams[0].timeouts = self.referee.blue.timeouts
        self.msg.teams[0].goalkeeper = self.referee.blue.goalkeeper
        self.msg.teams[0].red_cards = self.referee.blue.red_cards
        self.msg.teams[0].yellow_cards = self.referee.blue.yellow_cards

    def yellow_team_description(self):
        self.msg.teams[1].color = 'yellow'
        self.msg.teams[1].score = self.referee.yellow.score
        self.msg.teams[1].timeouts = self.referee.yellow.timeouts
        self.msg.teams[1].goalkeeper = self.referee.yellow.goalkeeper
        self.msg.teams[1].red_cards = self.referee.yellow.red_cards
        self.msg.teams[1].yellow_cards = self.referee.yellow.yellow_cards
