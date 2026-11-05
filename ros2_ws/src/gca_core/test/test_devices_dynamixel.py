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
        self.registers = {0: 1020, 6: 45, 64: 0, 11: 3, 44: 100, 98: 0, 104: 0, 128: -20}
        self.writes = []
        self.fail_address = None
        self.short_address = None
        self.wrong_readback = None

    def readTxRx(self, port, device_id, address, width):
        data = list(
            (self.registers.get(address, 0) & ((1 << (8 * width)) - 1)).to_bytes(width, "little")
        )
        if self.short_address == address:
            data = data[:-1]
        if self.wrong_readback == address:
            data[0] ^= 1
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
    assert [x[0] for x in packet.writes] == [64, 98, 11, 44, 98, 104]
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


def test_official_deadline_port_uses_monotonic_hard_limit():
    clock = Clock()
    factory, _ = dynamixel_factories(clock)
    port = factory("/never-opened")
    port.deadline_ns = clock() + 10
    port.setPacketTimeout(100)
    assert not port.isPacketTimeout()
    clock.now += 11
    assert port.isPacketTimeout()


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
