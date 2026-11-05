"""Real SerialEncoder code with a bounded serial test double (T02/T03/T07)."""

from dataclasses import replace

import pytest
from gca.devices.common import DeviceError, EncoderConfig
from gca.devices.encoder import SerialEncoder
from gca.protocols.encoder import append_crc


class Clock:
    def __init__(self):
        self.now = 1_000_000_000

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += max(1, round(seconds * 1e9))


class Serial:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)
        self.response = append_crc(bytes.fromhex("01 03 04 ff ff ff ff"))
        self.pending = bytearray()
        self.closed = False
        self.writes = []

    @property
    def in_waiting(self):
        return len(self.pending)

    def reset_input_buffer(self):
        self.pending.clear()

    def write(self, data):
        self.writes.append(data)
        self.pending.extend(self.response)
        return len(data)

    def read(self, count):
        data = bytes(self.pending[: min(count, 2)])
        del self.pending[: len(data)]
        return data

    def close(self):
        self.closed = True


def config():
    return EncoderConfig("/not-a-real-gca-encoder", 38400, 1, 0, 32, True, 0.002, verified=True)


@pytest.fixture
def encoder():
    clock = Clock()
    adapter = SerialEncoder(config(), clock=clock, sleeper=clock.sleep, serial_factory=Serial)
    yield adapter, clock
    adapter.close()


def test_open_is_read_only_and_each_valid_reply_has_new_sequence(encoder):
    adapter, clock = encoder
    identity = adapter.open()
    assert adapter.serial.writes == []
    first = adapter.read(clock() + 100_000_000)
    second = adapter.read(clock() + 100_000_000)
    assert first.raw_count == second.raw_count == -1
    assert (first.seq, second.seq) == (1, 2)
    assert first.device_epoch == identity.device_epoch
    assert first.request_started_ns <= first.received_ns
    assert adapter.serial.writes == [bytes.fromhex("01 03 00 00 00 02 c4 0b")] * 2


@pytest.mark.parametrize(
    "response",
    [
        b"",
        bytes.fromhex("01 03 04 00"),
        append_crc(bytes.fromhex("02 03 04 00 00 00 00")),
        append_crc(bytes.fromhex("01 83 02")),
        append_crc(bytes.fromhex("01 03 04 00 00 00 01")) * 2,
    ],
)
def test_bad_or_incomplete_response_requires_resync(encoder, response):
    adapter, clock = encoder
    adapter.open()
    adapter.serial.response = response
    with pytest.raises(DeviceError):
        adapter.read(clock() + 20_000_000)
    assert adapter.seq == 0
    with pytest.raises(DeviceError) as caught:
        adapter.read(clock() + 20_000_000)
    assert caught.value.code == "RECONNECT_REQUIRED"


def test_unsolicited_old_reply_is_not_new_feedback(encoder):
    adapter, clock = encoder
    adapter.open()
    adapter.serial.pending.extend(b"late")
    with pytest.raises(DeviceError) as caught:
        adapter.read(clock() + 100_000_000)
    assert caught.value.code == "STALE_RESPONSE"
    assert adapter.seq == 0


def test_reconnect_changes_epoch_and_resets_sequence(encoder):
    adapter, clock = encoder
    first = adapter.open()
    adapter.read(clock() + 100_000_000)
    adapter.close()
    second = adapter.open()
    assert first.device_epoch != second.device_epoch and adapter.seq == 0


def test_unverified_does_not_open_serial():
    adapter = SerialEncoder(
        replace(config(), verified=False),
        serial_factory=lambda **_: pytest.fail("must not connect"),
    )
    with pytest.raises(DeviceError):
        adapter.open()
