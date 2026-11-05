"""Optional dependency failure and pinned SDK socket setup without real network."""

import sys

import pytest
from gca.devices.common import DeviceError
from gca.devices.sdk_support import bounded_pigpio_client, dynamixel_factories


def test_missing_sdk_is_explicit(monkeypatch):
    monkeypatch.setitem(sys.modules, "dynamixel_sdk", None)
    with pytest.raises(DeviceError) as caught:
        dynamixel_factories(lambda: 123)
    assert caught.value.code == "DEPENDENCY_MISSING"


def test_missing_pigpio_is_explicit_without_connection(monkeypatch):
    monkeypatch.setitem(sys.modules, "pigpio", None)
    with pytest.raises(DeviceError) as caught:
        bounded_pigpio_client("localhost", 8888, 0.02)
    assert caught.value.code == "DEPENDENCY_MISSING"


def test_official_pigpio_wave_only_socket_has_connect_and_io_timeouts(monkeypatch):
    import socket

    calls = []

    class FakeSocket:
        def settimeout(self, timeout):
            calls.append(("timeout", timeout))

        def connect(self, address):
            calls.append(("connect", address))

        def setsockopt(self, *args):
            calls.append(("sockopt", args))

        def close(self):
            calls.append(("close",))

    def make_socket(family, kind):
        calls.append(("socket", family, kind))
        return FakeSocket()

    monkeypatch.setattr(socket, "socket", make_socket)
    monkeypatch.setattr(
        socket, "getaddrinfo", lambda *_args, **_kwargs: pytest.fail("DNS must never run")
    )
    client = bounded_pigpio_client("localhost", 8888, 0.02)
    assert client.connected and client._notify is None
    assert calls[0] == ("socket", socket.AF_INET, socket.SOCK_STREAM)
    assert ("connect", ("127.0.0.1", 8888)) in calls
    assert ("timeout", 0.02) in calls
    client.stop()
    assert ("close",) in calls


def test_pigpio_dns_host_refused_before_socket_or_lookup(monkeypatch):
    import socket

    monkeypatch.setattr(socket, "socket", lambda *_args: pytest.fail("must not connect"))
    monkeypatch.setattr(socket, "getaddrinfo", lambda *_args: pytest.fail("must not resolve"))
    with pytest.raises(DeviceError) as caught:
        bounded_pigpio_client("untrusted.example", 8888, 0.02)
    assert caught.value.code == "CONFIG_INVALID"
