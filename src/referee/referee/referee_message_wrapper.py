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

        # self.msg.teams[0].kick_offs = self.referee.blue.kick_offs
        # self.msg.teams[0].penalties = self.referee.blue.penalties
        # self.msg.teams[0].stoppage = self.referee.blue.stoppage
        # self.msg.teams[0].challenge_flag = self.referee.blue.challenge_flag
        # self.msg.teams[0].robot_substitution = self.referee.blue.robot_substitution
        # self.msg.teams[0].emergency_stop = self.referee.blue.emergency_stop
        # self.msg.teams[0].keeper_id = self.referee.blue.keeper_id
        # self.msg.teams[0].robots_currently_allowed = self.referee.blue.robots_currently_allowed
        # self.msg.teams[0].blue.robots_currently_on_the_field = self.referee.blue.robots_currently_on_the_field


    def yellow_team_description(self):
        self.msg.teams[1].color = 'yellow'
        self.msg.teams[1].score = self.referee.yellow.score
        self.msg.teams[1].timeouts = self.referee.yellow.timeouts
        self.msg.teams[1].goalkeeper = self.referee.yellow.goalkeeper
        self.msg.teams[1].red_cards = self.referee.yellow.red_cards
        self.msg.teams[1].yellow_cards = self.referee.yellow.yellow_cards

        # self.msg.teams[1].kick_offs = self.referee.yellow.kick_offs
        # self.msg.teams[1].penalties = self.referee.yellow.penalties
        # self.msg.teams[1].stoppage = self.referee.yellow.stoppage
        # self.msg.teams[1].challenge_flag = self.referee.yellow.challenge_flag
        # self.msg.teams[1].robot_substitution = self.referee.yellow.robot_substitution
        # self.msg.teams[1].emergency_stop = self.referee.yellow.emergency_stop
        # self.msg.teams[1].keeper_id = self.referee.yellow.keeper_id
        # self.msg.teams[1].robots_currently_allowed = self.referee.yellow.robots_currently_allowed
        # self.msg.teams[1].blue.robots_currently_on_the_field = self.referee.yellow.robots_currently_on_the_field
