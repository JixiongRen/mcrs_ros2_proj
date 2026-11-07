"""Held manual input and deadline checks; use simulated servo I/O only."""

import runpy
from dataclasses import replace
from pathlib import Path
from threading import Event, Thread
from time import monotonic, monotonic_ns

import pytest
from gca.domain.manual import ManualInputQuality, QualityError, QualityTracker, host_clock_id
from gca.domain.models import CommandContext, Lease
from gca.domain.servo import MANUAL_SERVO_VELOCITY, ServoJogCommand
from gca.runtime.feed_axis import FeedAxisRuntime
from gca.servo_config import (
    make_servo_settings,
    servo_settings_document,
    servo_settings_from_document,
)
from gca.simulation.servo import SimulatedVelocityDrive
from test_feed_axis_runtime import wait_for


def payload(mode="joydev_hold"):
    path = Path(__file__).resolve().parents[4] / "config" / "servo_settings.py"
    return runpy.run_path(str(path))["settings_payload"]("simulation", manual_input_mode=mode)


def quality(start=1_000_000_000, *, kind="JOYDEV_HELD_STATE"):
    return ManualInputQuality(
        kind,
        "unit-physical-source",
        "unit-epoch",
        10,
        start,
        "JOYDEV_INIT_ANCHORED_U32_MS",
        10,
        start,
        start + 150_000_000,
        host_clock_id(),
        "UNKNOWN",
    )


def test_schema2_hold_opt_in_uses_velocity_quantization_below_eighty_rpm():
    config = make_servo_settings(payload())
    assert config.schema_version == 2 and config.manual_hold_enabled
    assert config.device.max_velocity_raw == 349
    assert config.max_rpm == pytest.approx(79.921) and config.max_rpm <= 80
    old = payload("qualified_state")
    old["schema_version"] = 1
    del old["manual_input_mode"]
    legacy = make_servo_settings(old)
    document = servo_settings_document(legacy)
    assert not legacy.manual_hold_enabled
    assert servo_settings_from_document(document).sha256 == legacy.sha256
    assert set(document) == set(old) | {"sha256"}


def test_held_sources_require_opt_in_and_arm_proof_does_not_enter_motion_history():
    default = QualityTracker(backend="hardware")
    for kind in ("JOYDEV_HELD_STATE", "JOYDEV_HELD_ARM"):
        with pytest.raises(QualityError, match="HELD_INPUT_NOT_ENABLED"):
            default.validate(quality(kind=kind), 1_010_000_000)
    enabled = QualityTracker(backend="hardware", allow_joydev_hold=True)
    assert enabled.validate(quality(kind="JOYDEV_HELD_ARM"), 1_010_000_000)
    assert not enabled._history
    assert enabled.validate(quality(), 1_020_000_000)


def test_static_event_can_continue_only_with_advancing_reader_observations():
    tracker = QualityTracker(backend="hardware", allow_joydev_hold=True)
    first = quality()
    assert tracker.validate(first, 1_010_000_000)
    for index in range(1, 6):
        stamp = first.observation_ns + index * 100_000_000
        observed = replace(
            first,
            observation_seq=10 + index,
            observation_ns=stamp,
            valid_until_ns=stamp + 150_000_000,
        )
        assert tracker.validate(observed, stamp + 10_000_000)
        assert observed.event_seq == first.event_seq and observed.event_ns == first.event_ns
    with pytest.raises(QualityError, match="INPUT_OBSERVATION_REWRITTEN"):
        tracker.validate(replace(observed, valid_until_ns=observed.valid_until_ns - 1), stamp + 1)
    with pytest.raises(QualityError, match="INPUT_EXPIRED"):
        tracker.validate(observed, observed.valid_until_ns)


def test_neutral_init_proof_can_arm_but_cannot_be_used_as_motion_input():
    tracker = QualityTracker(backend="hardware", allow_joydev_hold=True)
    neutral = replace(quality(kind="JOYDEV_HELD_ARM"), event_seq=0, event_ns=0)
    assert tracker.validate(neutral, 1_010_000_000) == neutral.valid_until_ns
    with pytest.raises(QualityError, match="INPUT_TIME_INVALID"):
        tracker.validate(replace(neutral, source_kind="JOYDEV_HELD_STATE"), 1_010_000_000)


@pytest.fixture
def runtime():
    result = FeedAxisRuntime(make_servo_settings(payload()))
    assert result.start().accepted and result.activate().accepted
    assert result.arm(Lease("held-unit", 1, 1, 3.0, "executor", MANUAL_SERVO_VELOCITY)).accepted
    yield result
    result.inject_fault("control_freeze", False)
    result.close()


def command(sample, sequence=10, rpm=4.58):
    return ServoJogCommand(
        CommandContext("held-unit", "manual", f"held-{sequence}", 1, sequence),
        rpm,
        freshness_token="executor",
        ttl_s=0.15,
        quality=sample,
    )


def test_runtime_refuses_arm_proof_and_qualified_substitute_for_hold_mode(runtime):
    arm = quality(monotonic_ns(), kind="JOYDEV_HELD_ARM")
    assert runtime.jog(command(arm)).code == "ARM_INPUT_CANNOT_JOG"
    qualified = replace(
        quality(monotonic_ns()),
        source_kind="QUALIFIED_STATE_CONTRACT",
        link_evidence="QUALIFIED_STATE_CONTRACT",
        qualification_id="unit-not-real-qualification",
    )
    assert runtime.jog(command(qualified)).code == "INPUT_NOT_QUALIFIED"


def test_held_motion_continues_then_io_deadline_stops_when_control_freezes(runtime):
    first = quality(monotonic_ns())
    finish, sequence = monotonic() + 0.35, 10
    while monotonic() < finish:
        observed_ns = monotonic_ns()
        sample = replace(
            first,
            observation_seq=sequence,
            observation_ns=observed_ns,
            valid_until_ns=observed_ns + 150_000_000,
        )
        assert runtime.jog(command(sample, sequence)).accepted
        sequence += 1
        Event().wait(0.025)
    assert runtime.snapshot().servo.measured_velocity_rpm > 0
    before = runtime.simulation_evidence()["sample"]["seq"]
    assert runtime.inject_fault("control_freeze", True).accepted
    wait_for(lambda: not runtime.simulation_evidence()["sample"]["enabled"])
    assert runtime.simulation_evidence()["sample"]["seq"] > before
    assert runtime.inject_fault("control_freeze", False).accepted
    wait_for(lambda: runtime.snapshot().stop_progress == "CONFIRMED")
    assert not runtime.snapshot().servo.torque_enabled


def test_arm_serial_handshake_fits_original_input_deadline_without_duplicate_io(monkeypatch):
    from gca.devices.dynamixel import DynamixelVelocityDrive
    from test_devices_dynamixel import Clock, Packet, Port

    clock, transactions = Clock(), []

    class TimedPacket(Packet):
        def readTxRx(self, port, device_id, address, width):
            clock.now += 28_000_000
            transactions.append(("read", address, width, clock(), port.deadline_ns))
            return super().readTxRx(port, device_id, address, width)

        def writeTxRx(self, port, device_id, address, width, data):
            clock.now += 28_000_000
            transactions.append(("write", address, width, clock(), port.deadline_ns))
            return super().writeTxRx(port, device_id, address, width, data)

    values = payload()
    values["backend"] = "hardware"
    values["device"]["port"] = "/not-a-real-held-servo"
    config, packet = make_servo_settings(values), TimedPacket()
    packet.registers[128] = 0
    drive = DynamixelVelocityDrive(
        config.device, clock=clock, port_factory=Port, packet_factory=lambda: packet
    )
    runtime = FeedAxisRuntime(config, drive=drive, clock=clock)
    original = {}

    def arm_after_initial_feedback():
        runtime._state = replace(
            runtime._state, state="DISARMED", configured=True, stop_progress="CONFIRMED"
        )
        assert runtime._handle("active", True, clock()).accepted
        original["started"] = clock()
        original["deadline"] = clock() + 150_000_000
        lease = Lease("held-unit", 1, 1, 3.0, "executor", MANUAL_SERVO_VELOCITY)
        assert runtime._handle("arm", (lease, 0, original["deadline"]), clock()).accepted
        transactions.clear()

    # Exercise one real I/O-owner cycle deterministically; only the injected
    # transport clock advances, so operating-system scheduling adds no jitter.
    monkeypatch.setattr(runtime._ready, "set", arm_after_initial_feedback)
    monkeypatch.setattr(runtime._shutdown, "wait", lambda _: runtime._shutdown.set())
    runtime._io_owner()
    assert runtime._start_error is None and not runtime._emergency
    assert [entry[:3] for entry in transactions[:4]] == [
        ("write", 104, 4),
        ("read", 10, 102),
        ("write", 64, 1),
        ("read", 64, 1),
    ]
    assert sum(entry[:3] == ("read", 10, 102) for entry in transactions) == 1
    assert sum(address == 64 and data == b"\x01" for address, _, data in packet.writes) == 1
    assert transactions[3][3] - original["started"] == 112_000_000
    assert all(
        stamp < deadline <= original["deadline"] for _, _, _, stamp, deadline in transactions[:4]
    )
    assert runtime._arm_input_deadline == original["deadline"]


@pytest.fixture
def delayed_reenable(monkeypatch):
    values = payload()
    values["backend"] = "hardware"
    config = make_servo_settings(values)
    drive = SimulatedVelocityDrive(config)
    runtime = FeedAxisRuntime(config, drive=drive)
    entered, release, writes = Event(), Event(), []
    try:
        assert runtime.start().accepted and runtime.activate().accepted
        assert runtime.arm(
            Lease("held-unit", 1, 1, 3.0, "executor", MANUAL_SERVO_VELOCITY),
            input_deadline_ns=monotonic_ns() + 150_000_000,
        ).accepted
        wait_for(lambda: drive.enabled)
        assert runtime.stop("BEFORE_REENABLE").accepted
        wait_for(lambda: runtime.snapshot().stop_progress == "CONFIRMED")
        original_enable, original_write = drive.set_torque, drive.write_velocity_rpm

        def enable(enabled, deadline_ns=None, *, enable_guard=None):
            if enabled:
                entered.set()
                assert release.wait(0.8), "test did not release the simulated handshake"
            return original_enable(enabled, deadline_ns, enable_guard=enable_guard)

        def write(rpm, deadline_ns, *, write_guard=None):
            writes.append((rpm, monotonic_ns(), deadline_ns))
            return original_write(rpm, deadline_ns, write_guard=write_guard)

        monkeypatch.setattr(drive, "set_torque", enable)
        monkeypatch.setattr(drive, "write_velocity_rpm", write)
        yield runtime, entered, release, writes
    finally:
        release.set()
        runtime.close()


def test_reenable_admits_updates_during_160ms_handshake_and_writes_only_fresh_target(
    delayed_reenable,
):
    runtime, entered, release, writes = delayed_reenable
    first = quality(monotonic_ns())
    assert runtime.jog(command(first, rpm=1.0)).accepted
    assert entered.wait(0.2)
    finish, sequence = monotonic() + 0.160, 11
    while monotonic() < finish:
        stamp = monotonic_ns()
        updated = replace(
            first,
            observation_seq=sequence,
            observation_ns=stamp,
            valid_until_ns=stamp + 150_000_000,
        )
        assert runtime.jog(command(updated, sequence, rpm=-1.0)).accepted
        sequence += 1
        Event().wait(0.015)
    assert monotonic_ns() > first.valid_until_ns
    assert runtime._input_deadline == updated.valid_until_ns
    release.set()
    wait_for(lambda: bool(writes))
    assert all(rpm < 0 and stamp < deadline for rpm, stamp, deadline in writes)
    assert runtime.drive.enabled


def test_first_jog_during_explicit_arm_handshake_does_not_revoke_arm(delayed_reenable):
    runtime, entered, release, writes = delayed_reenable
    assert runtime.disarm().accepted
    wait_for(
        lambda: (
            runtime.snapshot().state == "DISARMED"
            and runtime.snapshot().stop_progress == "CONFIRMED"
        )
    )
    original_deadline = monotonic_ns() + 150_000_000
    assert runtime.arm(
        Lease("held-unit", 2, 1, 3.0, "executor", MANUAL_SERVO_VELOCITY),
        input_deadline_ns=original_deadline,
    ).accepted
    assert entered.wait(0.2)
    initial_generation = runtime._output_generation
    first = command(quality(monotonic_ns()), rpm=-1.0)
    first = replace(first, context=replace(first.context, generation=2))
    assert runtime.jog(first).accepted
    assert runtime._output_generation == initial_generation + 1
    release.set()
    wait_for(lambda: any(rpm < 0 for rpm, _, _ in writes))
    assert runtime.snapshot().armed and not runtime._emergency and runtime.drive.enabled
    assert runtime._arm_input_deadline == original_deadline
    assert all(stamp < deadline <= first.quality.valid_until_ns for _, stamp, deadline in writes)


@pytest.mark.parametrize("explicit_stop", [False, True])
def test_reenable_cannot_enable_after_input_expires_or_stop_during_handshake(
    delayed_reenable, explicit_stop
):
    runtime, entered, release, writes = delayed_reenable
    first = quality(monotonic_ns())
    assert runtime.jog(command(first, rpm=1.0)).accepted
    assert entered.wait(0.2)
    finish = monotonic() + 0.160
    if explicit_stop:
        Event().wait(0.080)
        stamp = monotonic_ns()
        updated = replace(
            first, observation_seq=11, observation_ns=stamp, valid_until_ns=stamp + 150_000_000
        )
        assert runtime.jog(command(updated, 11)).accepted
        assert runtime.stop("DURING_REENABLE").accepted
    Event().wait(max(0.0, finish - monotonic()))
    release.set()
    wait_for(lambda: runtime.snapshot().stop_progress == "CONFIRMED")
    assert not runtime.drive.enabled and not writes


@pytest.mark.parametrize("explicit_stop", [False, True])
def test_near_expiry_output_is_skipped_then_fresh_input_moves_and_stops(monkeypatch, explicit_stop):
    from test_devices_dynamixel import Clock

    clock, writes, reads = Clock(), [], []
    config = make_servo_settings(payload())
    budget_ns = int(config.io_timeout_s * 1e9)

    class TimedDrive(SimulatedVelocityDrive):
        def write_velocity_rpm(self, rpm, deadline_ns, *, write_guard=None):
            writes.append((rpm, clock(), deadline_ns))
            clock.now += 28_000_000
            return super().write_velocity_rpm(rpm, deadline_ns, write_guard=write_guard)

        def read_shaft_state(self, deadline_ns):
            reads.append((clock(), deadline_ns))
            clock.now += 28_000_000
            return super().read_shaft_state(deadline_ns)

    drive = TimedDrive(config, clock=clock)
    runtime = FeedAxisRuntime(config, drive=drive, clock=clock)
    original = {}

    def prepare_near_expiry_input():
        runtime._state = replace(
            runtime._state, state="DISARMED", configured=True, stop_progress="CONFIRMED"
        )
        assert runtime._handle("active", True, clock()).accepted
        lease = Lease("held-unit", 1, 1, 3.0, "executor", MANUAL_SERVO_VELOCITY)
        assert runtime._handle("arm", (lease, 0, 0), clock()).accepted
        sample = quality(clock() - 130_000_000)
        assert runtime._handle("jog", command(sample, rpm=-4.58), clock()).accepted
        runtime._last_output_ns = clock() - 40_000_000
        runtime._tick()
        original["expires_ns"] = runtime._demand.get().expires_ns
        assert original["expires_ns"] - clock() == 20_000_000

    cycles = 0

    def between_cycles(_):
        nonlocal cycles
        cycles += 1
        if cycles == 1:
            assert not writes
            assert runtime._demand.get().expires_ns == original["expires_ns"]
            assert not runtime._io_events.qsize()  # Skipping is not an accepted write.
            clock.now += 20_000_000
            observed = quality(clock())
            observed = replace(observed, event_seq=11, observation_seq=11)
            assert runtime._handle("jog", command(observed, 11, rpm=-4.58), clock()).accepted
            runtime._tick()
            original["fresh_expires_ns"] = runtime._demand.get().expires_ns
        elif cycles == 2:
            assert len(writes) == 1 and writes[0][0] < 0
            assert drive.enabled and drive._goal_rpm < 0
            if explicit_stop:
                runtime._latch_stop("UNIT_STOP")
            else:
                # No further inputs or control ticks: the I/O owner must stop.
                clock.now = original["fresh_expires_ns"] + 1
        else:
            assert len(writes) == 1 and not drive.enabled
            assert drive._goal_rpm == 0 and runtime.stop_serial > 0
            runtime._shutdown.set()

    monkeypatch.setattr(runtime._ready, "set", prepare_near_expiry_input)
    monkeypatch.setattr(runtime._shutdown, "wait", between_cycles)
    runtime._io_owner()
    assert cycles == 3 and runtime._start_error is None and not runtime._emergency
    assert all(deadline - started == budget_ns for started, deadline in reads)
    assert writes[0][2] - writes[0][1] == budget_ns
    assert writes[0][2] <= original["fresh_expires_ns"]


def test_control_updates_and_stop_do_not_wait_for_velocity_ack(monkeypatch):
    from test_devices_dynamixel import Clock

    clock, updated, finished, failures = Clock(), Event(), Event(), []
    config = make_servo_settings(payload())

    class DelayedAckDrive(SimulatedVelocityDrive):
        def write_velocity_rpm(self, rpm, deadline_ns, *, write_guard=None):
            accepted = super().write_velocity_rpm(rpm, deadline_ns, write_guard=write_guard)

            def control_during_ack():
                try:
                    sample = replace(quality(clock()), event_seq=11, observation_seq=11)
                    assert runtime._handle("jog", command(sample, 11, rpm=-1), clock()).accepted
                    clock.now += 10_000_000
                    runtime._tick()
                    updated.set()
                    runtime._latch_stop("B_DURING_ACK")
                except Exception as exc:
                    failures.append(exc)
                finally:
                    finished.set()

            worker = Thread(target=control_during_ack)
            worker.start()
            # The transport clock is fixed while waiting: this checks lock
            # ownership, not operating-system timing or a longer I/O budget.
            assert finished.wait(0.5), "control/STOP blocked behind ACK receive"
            worker.join()
            assert updated.is_set() and not failures
            assert not runtime._permit and not runtime._torque_requested
            return accepted  # This late receipt belongs to the revoked output.

    drive = DelayedAckDrive(config, clock=clock)
    runtime = FeedAxisRuntime(config, drive=drive, clock=clock)

    def prepare_motion():
        runtime._state = replace(
            runtime._state, state="DISARMED", configured=True, stop_progress="CONFIRMED"
        )
        assert runtime._handle("active", True, clock()).accepted
        lease = Lease("held-unit", 1, 1, 3.0, "executor", MANUAL_SERVO_VELOCITY)
        assert runtime._handle("arm", (lease, 0, 0), clock()).accepted
        assert runtime._handle("jog", command(quality(clock()), rpm=1), clock()).accepted
        runtime._last_output_ns = clock() - 40_000_000
        runtime._tick()

    def finish_cycle(_):
        runtime._tick()
        assert updated.is_set() and finished.is_set() and not failures
        assert runtime._accepted_rpm == 0 and drive._goal_rpm == 0
        assert not drive.enabled and not runtime._emergency
        runtime._shutdown.set()

    monkeypatch.setattr(runtime._ready, "set", prepare_motion)
    monkeypatch.setattr(runtime._shutdown, "wait", finish_cycle)
    runtime._io_owner()
    assert runtime._start_error is None


def test_delayed_held_input_fits_write_read_budget_without_renewing_original_deadline(monkeypatch):
    from test_devices_dynamixel import Clock

    clock, writes, waits = Clock(), [], []
    config = make_servo_settings(payload())

    class TimedDrive(SimulatedVelocityDrive):
        def write_velocity_rpm(self, rpm, deadline_ns, *, write_guard=None):
            writes.append((clock(), deadline_ns))
            clock.now += 28_000_000
            return super().write_velocity_rpm(rpm, deadline_ns, write_guard=write_guard)

        def read_shaft_state(self, deadline_ns):
            clock.now += 28_000_000
            return super().read_shaft_state(deadline_ns)

    drive = TimedDrive(config, clock=clock)
    runtime = FeedAxisRuntime(config, drive=drive, clock=clock)
    original_deadline = []

    def prepare_motion():
        runtime._state = replace(
            runtime._state, state="DISARMED", configured=True, stop_progress="CONFIRMED"
        )
        assert runtime._handle("active", True, clock()).accepted
        lease = Lease("held-unit", 1, 1, 3.0, "executor", MANUAL_SERVO_VELOCITY)
        assert runtime._handle("arm", (lease, 0, 0), clock()).accepted
        # The physical trace admitted one observation at age 111 ms.
        observed = clock() - 110_000_000
        sample = replace(quality(observed), valid_until_ns=observed + 300_000_000)
        original_deadline.append(sample.valid_until_ns)
        request = replace(command(sample, rpm=-config.max_rpm), ttl_s=config.output_ttl_s)
        assert runtime._handle("jog", request, clock()).accepted
        runtime._last_output_ns = clock() - 40_000_000
        runtime._tick()

    def between_cycles(wait_s):
        waits.append(wait_s)
        if len(waits) == 1:
            assert len(writes) == 1 and drive._goal_rpm < 0 and drive.enabled
            assert runtime._jog_deadline == original_deadline[0]
            assert writes[0][1] <= original_deadline[0]
            # WRITE + READ consumed 56 ms: no extra 20 ms idle interval.
            assert wait_s == 0
            clock.now = original_deadline[0] + 1
        else:
            assert len(writes) == 1 and not drive.enabled and drive._goal_rpm == 0
            runtime._shutdown.set()

    monkeypatch.setattr(runtime._ready, "set", prepare_motion)
    monkeypatch.setattr(runtime._shutdown, "wait", between_cycles)
    runtime._io_owner()
    assert len(waits) == 2 and runtime._start_error is None and not runtime._emergency
