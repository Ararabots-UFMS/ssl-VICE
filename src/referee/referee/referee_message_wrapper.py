from system_interfaces.msg import GameData
from referee.proto.ssl_gc_referee_message_pb2 import Referee
class MessageWrapping():
    def __init__(self, referee_message):
        self.stage = Referee.Stage.Name(referee_message.stage)
        self.command = Referee.Command.Name(referee_message.command)
        self.command_counter = referee_message.command_counter
    def to_game_data(self):
        # TODO: create a class to wrapper those messages
        msg = GameData()
        msg.stage = self.stage
        msg.command = self.command
        msg.command_counter = self.command_counter
        return msg