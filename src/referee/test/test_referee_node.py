from unittest.mock import MagicMock, patch

import pytest

from referee.referee.referee_node import RefereeNode


@pytest.fixture
def node():
    instance = RefereeNode.__new__(RefereeNode)
    instance._last_command = None
    instance._last_command_counter = None
    instance.publisher = MagicMock()
    instance.get_logger = MagicMock(return_value=MagicMock())
    return instance


def test_command_was_updated_detects_state_change(node):
    proto = MagicMock()
    proto.command = 1
    proto.command_counter = 10

    assert node.command_was_updated(proto) is True

    node._last_command = 1
    node._last_command_counter = 10
    assert node.command_was_updated(proto) is False

    proto.command = 2
    assert node.command_was_updated(proto) is True


def test_log_changes_logs_when_command_or_counter_changes(node):
    proto = MagicMock()
    proto.command = 3
    proto.command_counter = 42
    proto.stage = 7
    proto.stage_time_left = 30

    node.get_logger = MagicMock(return_value=MagicMock())
    with patch.object(node, "command_was_updated", return_value=True):
        node.log_changes(proto, (b"payload", ("127.0.0.1", 11003)))

    node.get_logger().info.assert_any_call("Received 7 bytes from ('127.0.0.1', 11003)")
    assert node._last_command == 3
    assert node._last_command_counter == 42


def test_publish_to_topic_wraps_and_publishes(node):
    proto = MagicMock()
    wrapped_message = MagicMock()

    with patch("referee.referee.referee_node.RefereeMessageWrapper", return_value=MagicMock(msg=wrapped_message)) as wrapper_cls:
        node.publish_to_topic(node, proto)

    wrapper_cls.assert_called_once_with(proto)
    node.publisher.publish.assert_called_once_with(wrapped_message)


def test_receive_and_publish_parses_and_publishes_message(node):
    node.client = MagicMock()
    node.client.receive.return_value = (b"payload", ("127.0.0.1", 11003))

    proto = MagicMock()
    with patch("referee.referee.referee_node.Referee", return_value=proto), \
         patch.object(node, "publish_to_topic") as publish_mock, \
         patch.object(node, "log_changes") as log_mock:
        node.receive_and_publish()

    proto.ParseFromString.assert_called_once_with(b"payload")
    publish_mock.assert_called_once_with(node, proto)
    log_mock.assert_called_once_with(proto, (b"payload", ("127.0.0.1", 11003)))


def test_receive_and_publish_skips_empty_results(node):
    node.client = MagicMock()
    node.client.receive.return_value = None

    with patch.object(node, "publish_to_topic") as publish_mock, \
         patch.object(node, "log_changes") as log_mock:
        node.receive_and_publish()

    publish_mock.assert_not_called()
    log_mock.assert_not_called()
    node.get_logger().warn.assert_called_once_with("No referee message received.")
