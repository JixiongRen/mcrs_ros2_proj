"""Native servo contracts and error paths; injected transport only."""

import importlib.util
import json
import math
from contextlib import contextmanager
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest
from gca.devices.common import DeviceError
from gca.devices.dynamixel import DynamixelVelocityDrive
from gca.devices.sdk_support import dynamixel_factories
from gca.servo_config import (
    load_servo_settings,
    make_servo_settings,
    servo_settings_document,
)
from gca.simulation.servo import SimulatedVelocityDrive
from test_devices_dynamixel import Clock, Packet, Port


def settings():
    script = Path(__file__).resolve().parents[4] / "config/servo_settings.py"
    spec = importlib.util.spec_from_file_location("servo_editable_settings", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return make_servo_settings(module.settings_payload("simulation"))


def test_configuration_roundtrip_without_ros_or_sdk_open_and_rejects_tampering(tmp_path):
    config = settings()
    target = tmp_path / "config.json"
    target.write_text(json.dumps(servo_settings_document(config)))
    assert load_servo_settings(target) == config
    assert config.max_rpm == pytest.approx(79.921)
    assert config.manual_axis_to_servo_sign == -1
    document = servo_settings_document(config)
    document["device"]["max_velocity_raw"] += 1
    target.write_text(json.dumps(document))
    with pytest.raises(ValueError, match="digest"):
        load_servo_settings(target)


@pytest.mark.parametrize(
    "field,value",
    [
        ("control_period_s", True),
        ("output_ttl_s", 0.05),
        ("output_ttl_s", 0.15),  # One transaction fits; the full write/read cycle does not.
        ("manual_input_max_age_s", 0.15),
        ("feedback_max_age_s", 0.15),
        ("manual_axis_to_servo_sign", 1),
        ("max_acceleration_rpm_s", float("nan")),
        ("mechanical_mapping", {"mm_per_revolution": 1}),
        ("unknown", 1),
    ],
)
def test_configuration_rejects_invalid_or_unimplemented_values(field, value):
    payload = servo_settings_document(settings())
    del payload["sha256"]
    payload[field] = value
    with pytest.raises(ValueError):
        make_servo_settings(payload)


@pytest.fixture
def native():
    clock, packet = Clock(), Packet()
    config = replace(settings().device, port="/not-a-real-native-servo")
    drive = DynamixelVelocityDrive(
        config, clock=clock, port_factory=Port, packet_factory=lambda: packet
    )
    yield drive, packet, clock
    packet.fail_address = packet.short_address = packet.wrong_readback = None
    drive.close()


def test_readonly_close_has_no_writes_and_shaft_feedback_has_units(native):
    drive, packet, clock = native
    drive.open_identified()
    sample = drive.read_shaft_state(clock() + 100_000_000)
    assert sample.raw_position == 4096 and sample.angle_rad == pytest.approx(2 * math.pi)
    assert sample.velocity_rpm == pytest.approx(-4.58) and sample.seq == 1
    assert sample.request_started_ns <= sample.received_ns
    drive.disconnect_readonly()
    assert packet.writes == [] and drive.port is None


def test_velocity_initialization_readbacks_no_redundant_eeprom_and_native_sign(native):
    drive, packet, clock = native
    drive.open_identified()
    drive.configure_velocity_mode()
    assert packet.registers[11] == 1 and packet.registers[108] == 12
    assert packet.registers[98] == 25 and packet.registers[104] == 0
    packet.writes.clear()
    drive.configure_velocity_mode()
    assert all(address not in (11, 44) for address, _, _ in packet.writes)
    drive.set_torque(True)
    assert drive.write_velocity_rpm(-2, clock() + 100_000_000) == pytest.approx(-2.061)
    assert packet.registers[104] == -9
    with pytest.raises(DeviceError, match="stop/close"):
        drive.disconnect_readonly()


def test_watchdog_latch_is_preserved_and_only_explicit_disabled_recovery_clears_it(native):
    drive, packet, clock = native
    drive.open_identified()
    packet.registers[98] = -1
    with pytest.raises(DeviceError) as caught:
        drive.configure_velocity_mode()
    assert caught.value.code == "WATCHDOG_LATCHED"
    assert packet.registers[98] == -1 and not drive.configured
    assert not any(address == 98 for address, _, _ in packet.writes)
    sample = drive.read_shaft_state(clock() + 100_000_000)
    assert sample.valid and sample.error_code == "WATCHDOG_LATCHED" and sample.watchdog_raw == -1
    drive.recover_watchdog()
    assert packet.registers[98] == 25 and packet.registers[104] == 0
    assert not drive.configured and packet.registers[64] == 0


def test_changed_configuration_between_configure_and_arm_cannot_enable(native):
    drive, packet, clock = native
    drive.open_identified()
    drive.configure_velocity_mode()
    packet.registers[10] = 1
    packet.writes.clear()
    with pytest.raises(DeviceError) as caught:
        drive.set_torque(True)
    assert caught.value.code == "CONFIG_READBACK_MISMATCH"
    assert not drive.configured and not any(
        a == 64 and data == b"\x01" for a, _, data in packet.writes
    )


def test_shared_deadline_does_not_renew_for_following_read(native):
    drive, packet, clock = native
    drive.open_identified()
    drive.configure_velocity_mode()
    drive.set_torque(True)
    deadline = clock() + 20_000_000
    drive.write_velocity_rpm(1, deadline)
    clock.now = deadline
    with pytest.raises(DeviceError) as caught:
        drive.read_shaft_state(deadline)
    assert caught.value.code == "DEVICE_TIMEOUT"


@pytest.mark.parametrize("revoke", [False, True])
def test_velocity_guard_covers_wire_write_only_and_cancellation_allows_stop(revoke):
    clock, writes, receiving, guard_active = Clock(), [], [], []
    port_factory, packet_factory = dynamixel_factories(clock)
    drive = DynamixelVelocityDrive(settings().device, clock=clock)
    drive.port, drive.packet = port_factory("/never-opened"), packet_factory()
    drive.configured = drive.enabled = True
    reply = bytearray()

    @contextmanager
    def guard():
        if revoke:
            raise DeviceError("OUTPUT_SUPERSEDED")
        guard_active.append(True)
        try:
            yield
        finally:
            guard_active.clear()

    def write(data):
        # The first permitted write is motion; STOP has no motion guard.
        if drive.port.write_guard is not None:
            assert guard_active
        writes.append(bytes(data))
        reply.extend(bytes.fromhex("ff ff fd 00 01 04 00 55 00 a1 0c"))
        return len(data)

    def read(length):
        assert not guard_active, "ACK receive must not hold the control fence"
        receiving.append(length)
        part = bytes(reply[:length])
        del reply[:length]
        return part

    drive.port.ser = SimpleNamespace(
        write=write,
        read=read,
        reset_input_buffer=reply.clear,
        write_timeout=None,
        timeout=None,
    )
    deadline = clock() + 80_000_000
    if revoke:
        with pytest.raises(DeviceError, match="OUTPUT_SUPERSEDED"):
            drive.write_velocity_rpm(-2, deadline, write_guard=guard)
        assert not writes and not receiving
    else:
        assert drive.write_velocity_rpm(-2, deadline, write_guard=guard) == pytest.approx(-2.061)
        assert len(writes) == 1 and receiving
    assert not drive.port.is_using and drive.port.write_guard is None
    drive.stop(deadline)
    assert writes[-1][8:10] == b"\x68\x00"  # Goal Velocity address 104.
    assert writes[-1][10:14] == bytes(4)
    assert not drive.port.is_using


def test_stop_during_arm_configuration_read_never_writes_torque_enable(native):
    drive, packet, clock = native
    drive.open_identified()
    drive.configure_velocity_mode()
    packet.writes.clear()
    allowed = [True]
    original = packet.readTxRx

    def stop_during_read(port, device_id, address, width):
        reply = original(port, device_id, address, width)
        if address == 10 and width == 102:
            allowed[0] = False
        return reply

    packet.readTxRx = stop_during_read
    with pytest.raises(DeviceError) as caught:
        drive.set_torque(True, enable_guard=lambda: allowed[0])
    assert caught.value.code == "STOP_SUPERSEDED"
    assert not any(address == 64 and data == b"\x01" for address, _, data in packet.writes)
    assert packet.registers[64] == 0


def test_exclusive_is_given_to_serial_constructor_before_open(monkeypatch):
    calls = []

    def serial_factory(**kwargs):
        calls.append(kwargs)
        return SimpleNamespace(reset_input_buffer=lambda: None)

    monkeypatch.setattr("serial.Serial", serial_factory)
    port_factory, _ = dynamixel_factories(Clock())
    assert port_factory("/not-a-device").setBaudRate(57600)
    assert calls[0]["exclusive"] is True


def test_simulation_responds_to_accepted_output_and_bus_watchdog_is_not_command_watchdog():
    config, clock = settings(), Clock()
    drive = SimulatedVelocityDrive(config, clock=clock)
    drive.open_identified()
    drive.configure_velocity_mode()
    drive.set_torque(True)
    accepted = drive.write_velocity_rpm(-4.0, clock() + 100_000_000)
    immediate = drive.read_shaft_state(clock() + 100_000_000)
    assert immediate.velocity_rpm == 0 and accepted < 0
    for _ in range(5):
        clock.now += 100_000_000
        sample = drive.read_shaft_state(clock() + 100_000_000)
    assert accepted < sample.velocity_rpm < 0 and sample.raw_position < 0
    assert sample.watchdog_raw == config.device.watchdog_raw
    clock.now += 600_000_000
    sample = drive.read_shaft_state(clock() + 100_000_000)
    assert sample.watchdog_raw == -1 and sample.error_code == "WATCHDOG_LATCHED"
    assert sample.valid  # Physical feedback remains inspectable despite the latched fault.
    drive.set_torque(False)
    drive.recover_watchdog()
    drive.close()
