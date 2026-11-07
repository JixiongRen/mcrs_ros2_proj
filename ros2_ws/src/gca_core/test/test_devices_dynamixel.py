"""Real adapter/official SDK paths with injected transport, never a device (T04/T07)."""

from dataclasses import replace
from types import SimpleNamespace

import pytest
from gca.devices.common import DeviceError, DynamixelConfig
from gca.devices.dynamixel import DynamixelServo
from gca.devices.sdk_support import dynamixel_factories


class Clock:
    def __init__(self):
        self.now = 1_000_000_000

    def __call__(self):
        return self.now


class Port:
    def __init__(self, name):
        self.name = name
        self.is_open = False
        self.ser = SimpleNamespace(timeout=None, write_timeout=None)

    def setBaudRate(self, baud):
        self.is_open = True
        return True

    def closePort(self):
        self.is_open = False


class Packet:
    def __init__(self):
        self.registers = {
            0: 1020,
            6: 45,
            10: 0,
            11: 3,
            13: 2,
            44: 100,
            64: 0,
            70: 0,
            98: 0,
            104: 0,
            108: 1,
            128: -20,
            132: 4096,
        }
        self.writes = []
        self.fail_address = None
        self.short_address = None
        self.wrong_readback = None

    def readTxRx(self, port, device_id, address, width):
        memory = bytearray(148)
        widths = {0: 2, 44: 4, 104: 4, 108: 4, 128: 4, 132: 4}
        for register, value in self.registers.items():
            size = widths.get(register, 1)
            memory[register : register + size] = (value & ((1 << (8 * size)) - 1)).to_bytes(
                size, "little"
            )
        data = list(memory[address : address + width])
        if self.short_address is not None and address <= self.short_address < address + width:
            data = data[:-1]
        if self.wrong_readback is not None and address <= self.wrong_readback < address + width:
            data[self.wrong_readback - address] ^= 1
        return data, 0, 0

    def writeTxRx(self, port, device_id, address, width, data):
        self.writes.append((address, width, bytes(data)))
        if address == self.fail_address:
            return -1001, 0
        self.registers[address] = int.from_bytes(bytes(data), "little", signed=address == 104)
        return 0, 0


def config():
    return DynamixelConfig(
        "/not-a-real-gca-device", 57600, 1, 1020, 40.0, 0.229, 500, 3, verified=True
    )


@pytest.fixture
def servo():
    packet, clock = Packet(), Clock()
    adapter = DynamixelServo(
        config(), clock=clock, port_factory=Port, packet_factory=lambda: packet
    )
    yield adapter, packet, clock
    packet.fail_address = packet.short_address = packet.wrong_readback = None
    adapter.close()


def test_construct_and_open_do_not_enable_or_configure(servo):
    adapter, packet, clock = servo
    assert adapter.port is None and packet.writes == []
    identity = adapter.open()
    assert identity.model == "DYNAMIXEL:1020"
    assert packet.writes == []
    assert adapter.port.ser.timeout == 0
    assert adapter.port.ser.write_timeout > 0


def test_configure_readback_zero_before_enable_and_signed_quantization(servo):
    adapter, packet, clock = servo
    adapter.open()
    adapter.configure()
    assert [x[0] for x in packet.writes] == [64, 11, 44, 98, 104]
    assert not adapter.enabled and packet.registers[64] == 0
    adapter.enable(True)
    assert [x[0] for x in packet.writes[-2:]] == [104, 64]
    applied = adapter.write_velocity(-2.0, clock() + 100_000_000)
    assert packet.registers[104] < 0
    assert applied == pytest.approx(-2.0, abs=0.1)
    sample = adapter.read_state(clock() + 100_000_000)
    assert sample.velocity_mm_s < 0 and sample.enabled


@pytest.mark.parametrize("address", [64, 98, 11, 44, 98, 104])
def test_every_configuration_failure_prevents_enable(servo, address):
    adapter, packet, clock = servo
    adapter.open()
    packet.fail_address = address
    with pytest.raises(DeviceError):
        adapter.configure()
    assert not adapter.configured
    with pytest.raises(DeviceError, match="before"):
        adapter.enable(True)
    assert not any(a == 64 and d == b"\x01" for a, _, d in packet.writes)


def test_wrong_readback_refuses_enable(servo):
    adapter, packet, clock = servo
    adapter.open()
    packet.wrong_readback = 11
    with pytest.raises(DeviceError) as caught:
        adapter.configure()
    assert caught.value.code == "CONFIG_READBACK_MISMATCH"
    assert not adapter.configured


def test_limits_timeout_and_invalid_measurement_are_explicit(servo):
    adapter, packet, clock = servo
    adapter.open()
    adapter.configure()
    adapter.enable(True)
    for value in (float("nan"), float("inf"), 1e9):
        with pytest.raises(DeviceError):
            adapter.write_velocity(value, clock() + 100_000_000)
    with pytest.raises(DeviceError):
        adapter.write_velocity(1.0, clock() - 1)
    packet.short_address = 128
    with pytest.raises(DeviceError) as caught:
        adapter.read_state(clock() + 100_000_000)
    assert caught.value.code == "BAD_FRAME"


def test_identity_failure_closes_port(servo):
    adapter, packet, clock = servo
    packet.registers[0] = 999
    with pytest.raises(DeviceError):
        adapter.open()
    assert adapter.port is None


def test_unverified_refuses_connection():
    adapter = DynamixelServo(
        replace(config(), verified=False), port_factory=lambda *_: pytest.fail("must not open")
    )
    with pytest.raises(DeviceError) as caught:
        adapter.open()
    assert caught.value.code == "HARDWARE_UNVERIFIED"


def test_official_sdk_checked_packet_rejects_legal_crc_without_data():
    _, handler = dynamixel_factories(Clock())
    packet = handler()

    class WirePort:
        is_using = False

        def __init__(self):
            self.reply = list(bytes.fromhex("ff ff fd 00 01 04 00 55 00 a1 0c"))

        def clearPort(self):
            return None

        def writePort(self, data):
            return len(data)

        def setPacketTimeout(self, length):
            return None

        def readPort(self, length):
            part, self.reply = self.reply[:length], self.reply[length:]
            return part

        def isPacketTimeout(self):
            return not self.reply

    # Even a one-byte read must not accept the first CRC octet as measurement.
    data, result, error = packet.readTxRx(WirePort(), 1, 132, 1)
    assert result != 0 and data == []


def test_official_sdk_checked_packet_accepts_stuffed_data_only_with_valid_wire_crc():
    from dynamixel_sdk import COMM_RX_CORRUPT, COMM_SUCCESS

    _, handler = dynamixel_factories(Clock())
    packet = handler()
    payload = [0xAA, 0xFF, 0xFF, 0xFD, 0x12]
    reply = packet.addStuffing(
        [0xFF, 0xFF, 0xFD, 0, 1, len(payload) + 4, 0, 0x55, 0, *payload, 0, 0]
    )
    crc = packet.updateCRC(0, reply, len(reply) - 2)
    reply[-2:] = [crc & 0xFF, crc >> 8]
    assert len(reply) == 11 + len(payload) + 1

    class WirePort:
        is_using = False

        def __init__(self, reply):
            self.reply = list(reply)

        def clearPort(self):
            return None

        def writePort(self, data):
            return len(data)

        def setPacketTimeout(self, length):
            return None

        def readPort(self, length):
            part, self.reply = self.reply[:length], self.reply[length:]
            return part

        def isPacketTimeout(self):
            return not self.reply

    data, result, error = packet.readTxRx(WirePort(reply), 1, 132, len(payload))
    assert (data, result, error) == (payload, COMM_SUCCESS, 0)
    reply[-1] ^= 1
    data, result, error = packet.readTxRx(WirePort(reply), 1, 132, len(payload))
    assert result == COMM_RX_CORRUPT and data == []


def test_official_deadline_port_uses_monotonic_hard_limit():
    clock = Clock()
    factory, _ = dynamixel_factories(clock)
    port = factory("/never-opened")
    port.deadline_ns = clock() + 10
    port.setPacketTimeout(100)
    assert not port.isPacketTimeout()
    clock.now += 11
    assert port.isPacketTimeout()


def test_deadline_serial_read_waits_only_within_remaining_budget():
    clock, calls = Clock(), []
    factory, _ = dynamixel_factories(clock)
    port = factory("/never-opened")
    port.deadline_ns = clock() + 80_000_000

    def read(length):
        calls.append((length, port.ser.timeout))
        clock.now += 30_000_000
        return b"\x01"

    port.ser = SimpleNamespace(timeout=0, read=read)
    assert port.readPort(11) == b"\x01"
    assert port.readPort(72) == b"\x01"
    assert calls == [(11, 0.08), (72, 0.05)]
    clock.now = port.deadline_ns
    assert port.readPort(1) == b"" and len(calls) == 2


@pytest.mark.parametrize("tail_ms,corrupt_crc", [(55, False), (55, True), (90, False)])
def test_sdk_partial_response_uses_configured_deadline_and_preserves_crc(tail_ms, corrupt_crc):
    from dynamixel_sdk import COMM_RX_CORRUPT, COMM_SUCCESS

    clock = Clock()
    factory, handler = dynamixel_factories(clock)
    packet = handler()
    payload = list(range(72))
    reply = [0xFF, 0xFF, 0xFD, 0, 1, 76, 0, 0x55, 0, *payload, 0, 0]
    crc = packet.updateCRC(0, reply, len(reply) - 2)
    reply[-2:] = [crc & 0xFF, crc >> 8]
    if corrupt_crc:
        reply[-1] ^= 1
    started = clock()

    class DelayedPort(factory):
        def __init__(self):
            super().__init__("/never-opened")
            self.tx_time_per_byte = 1000 * 10 / 57600
            self.deadline_ns = started + 80_000_000
            self.offset = 0

        def clearPort(self):
            pass

        def writePort(self, data):
            return len(data)

        def readPort(self, length):
            clock.now += 1_000_000
            available = len(reply) if clock() >= started + tail_ms * 1_000_000 else 30
            end = min(self.offset + length, available)
            part = reply[self.offset : end]
            self.offset = end
            return part

    port = DelayedPort()
    data, result, error = packet.readTxRx(port, 1, 64, len(payload))
    if tail_ms < 80:
        assert clock() >= started + tail_ms * 1_000_000
        assert clock() < port.deadline_ns
        assert result == (COMM_RX_CORRUPT if corrupt_crc else COMM_SUCCESS)
        assert data == ([] if corrupt_crc else payload)
    else:
        assert clock() >= port.deadline_ns
        assert result == COMM_RX_CORRUPT and data == []
    assert error == 0


def test_unknown_model_never_connects_or_writes():
    adapter = DynamixelServo(
        replace(config(), model_number=12),
        port_factory=lambda *_: pytest.fail("must not connect"),
        packet_factory=Packet,
    )
    with pytest.raises(DeviceError) as caught:
        adapter.open()
    assert caught.value.code == "MODEL_UNSUPPORTED"


def test_old_firmware_has_no_watchdog_capability_and_never_enables(servo):
    adapter, packet, clock = servo
    packet.registers[6] = 37
    with pytest.raises(DeviceError) as caught:
        adapter.open()
    assert caught.value.code == "CAPABILITY_UNAVAILABLE"
    assert not packet.writes and adapter.port is None


def test_watchdog_cannot_be_disabled_in_hardware_configuration():
    with pytest.raises(ValueError):
        replace(config(), watchdog_raw=0)


def test_failed_zero_still_attempts_independent_torque_disable(servo):
    adapter, packet, clock = servo
    adapter.open()
    adapter.configure()
    adapter.enable(True)
    packet.writes.clear()
    packet.fail_address = 104
    with pytest.raises(DeviceError):
        adapter.close()
    assert [address for address, _, _ in packet.writes] == [104, 64]
    assert packet.registers[64] == 0
    assert adapter.port is None
