import socket

import pytest

from referee.referee.referee_client import RefereeClient


class DummySocket:
    def __init__(self):
        self.calls = []
        self.bind_addr = None
        self.timeout = None

    def setsockopt(self, level, optname, value):
        self.calls.append((level, optname, value))

    def bind(self, addr):
        self.bind_addr = addr

    def settimeout(self, timeout):
        self.timeout = timeout


def test_connect_sets_up_multicast_socket_with_default_interface(monkeypatch):
    dummy = DummySocket()

    monkeypatch.setattr(
        "referee.referee.referee_client.socket.socket",
        lambda *args, **kwargs: dummy,
    )
    monkeypatch.setattr(
        "referee.referee.referee_client.socket.inet_aton",
        lambda ip: {"0.0.0.0": b"\x00\x00\x00\x00", "224.5.23.1": b"\xe0\x05\x17\x01"}[ip],
    )
    monkeypatch.setattr(
        "referee.referee.referee_client.struct.pack",
        lambda fmt, group, local_ip: b"mreq",
    )

    client = RefereeClient("224.5.23.1", 11003, 65536)
    client.connect()

    assert client.sock is dummy
    assert dummy.bind_addr == ("", 11003)
    assert (socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) in dummy.calls
    assert (socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, b"mreq") in dummy.calls


def test_connect_uses_explicit_interface(monkeypatch):
    dummy = DummySocket()

    monkeypatch.setattr(
        "referee.referee.referee_client.socket.socket",
        lambda *args, **kwargs: dummy,
    )
    monkeypatch.setattr(
        "referee.referee.referee_client.socket.inet_aton",
        lambda ip: {"127.0.0.1": b"\x7f\x00\x00\x01", "224.5.23.1": b"\xe0\x05\x17\x01"}[ip],
    )
    monkeypatch.setattr(
        "referee.referee.referee_client.struct.pack",
        lambda fmt, group, local_ip: b"explicit-interface",
    )

    client = RefereeClient("224.5.23.1", 11003, 65536)
    client.connect(interface_ip="127.0.0.1")

    assert (socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, b"explicit-interface") in dummy.calls


def test_receive_returns_none_on_timeout(monkeypatch):
    class TimedOutSocket:
        def recvfrom(self, _):
            raise socket.timeout

    monkeypatch.setattr(
        "referee.referee.referee_client.socket.socket",
        lambda *args, **kwargs: TimedOutSocket(),
    )

    client = RefereeClient("224.5.23.1", 11003, 65536)
    client.sock = TimedOutSocket()

    assert client.receive() is None


def test_receive_raises_runtime_error_on_socket_failures(monkeypatch):
    class BrokenSocket:
        def recvfrom(self, _):
            raise OSError("boom")

    client = RefereeClient("224.5.23.1", 11003, 65536)
    client.sock = BrokenSocket()

    with pytest.raises(RuntimeError, match="Error receiving multicast message: boom"):
        client.receive()


def test_local_only_skips_packets_from_other_machines():
    """Another machine's game controller on the same group must not steer our robots."""
    class TwoSenders:
        def __init__(self):
            self.queue = [(b"theirs", ("10.6.251.133", 11003)), (b"ours", ("127.0.0.1", 11003))]

        def recvfrom(self, _):
            return self.queue.pop(0)

    client = RefereeClient("224.5.23.1", 11003, 65536, local_only=True)
    client.sock = TwoSenders()
    client._is_local = {"10.6.251.133": False, "127.0.0.1": True}

    assert client.receive() == (b"ours", ("127.0.0.1", 11003))


def test_without_local_only_every_sender_is_accepted():
    class OneSender:
        def recvfrom(self, _):
            return b"theirs", ("10.6.251.133", 11003)

    client = RefereeClient("224.5.23.1", 11003, 65536)
    client.sock = OneSender()

    assert client.receive() == (b"theirs", ("10.6.251.133", 11003))


def test_source_ip_and_local_only_must_both_pass(monkeypatch):
    """A local packet from the wrong address, or the right address seen remotely, is dropped."""
    class ThreeSenders:
        def __init__(self):
            self.queue = [(b"local, other ip", ("127.0.0.1", 11003)),
                          (b"remote", ("10.6.251.133", 11003)),
                          (b"ours", ("10.6.0.5", 11003))]

        def recvfrom(self, _):
            return self.queue.pop(0)

    monkeypatch.setenv("ARARABOTS_SIM_SOURCE_IP", "10.6.0.5")
    client = RefereeClient("224.5.23.1", 11003, 65536, local_only=True)
    client.sock = ThreeSenders()
    client._is_local = {"127.0.0.1": True, "10.6.251.133": False, "10.6.0.5": True}

    assert client.receive() == (b"ours", ("10.6.0.5", 11003))
