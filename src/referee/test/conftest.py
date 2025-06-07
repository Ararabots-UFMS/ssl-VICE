import sys
from unittest.mock import MagicMock

# File of mocks for the referee tests

# Mock for the system_interfaces module
mock_system_interfaces = MagicMock()
sys.modules["system_interfaces"] = mock_system_interfaces
sys.modules["system_interfaces.msg"] = mock_system_interfaces.msg


# Mock for the RefereeMessage
def mock_referee_message():
    msg = MagicMock(name="RefereeMessage")
    msg.teams = []
    return msg

mock_system_interfaces.msg.RefereeMessage.side_effect = mock_referee_message

# Mock for the TeamData
mock_system_interfaces.msg.TeamData.side_effect = lambda: MagicMock()


# Mock for the Referee module
mock_proto_ref = MagicMock()
sys.modules["referee.proto.ssl_gc_referee_message_pb2"] = mock_proto_ref

# Mock for the Referee returning the stage and command names functions
mock_proto_ref.Referee.Stage.Name = MagicMock(return_value="NORMAL_FIRST_HALF")
mock_proto_ref.Referee.Command.Name = MagicMock(return_value="HALT")
