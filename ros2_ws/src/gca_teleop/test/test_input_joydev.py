"""Injected joydev syscalls: no device, daemon, haptics or permissions touched."""

import errno
import json
import select
import struct
from dataclasses import replace
from pathlib import Path

import pytest
from gca_teleop.config import load_profile
from gca_teleop.input.joydev import NativeJoydev, discover, inspect_device
from gca_teleop.input.scripted import ScriptedInput, load_script
from gca_teleop.model import InputError, InputIdentity

CONFIG = Path(__file__).parents[1] / "config"


class Clock:
    def __init__(self):
        self.now = 10_000_000_000

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += int(seconds * 1e9)


class Kernel:
    def __init__(self):
        self.clock = Clock()
        self.device = InputIdentity(
            "/dev/input/js7",
            "fixture",
            0x20BC,
            0x506D,
            3,
            1,
            (0, 1, 2),
            (304, 305),
            device_mode="fixture",
        )
        self.chunks, self.read_sizes, self.open_flags, self.closed = [], [], [], []
        self.flags = select.POLLIN

    def open(self, path, flags):
        self.open_flags.append(flags)
        return 17

    def close(self, fd):
        self.closed.append(fd)

    def ioctl(self, fd, code, value, mutate=True):
        if code == 0x80016A11:
            value[0] = len(self.device.axis_codes)
        elif code == 0x80016A12:
            value[0] = len(self.device.button_codes)
        elif code == 0x80406A32:
            value[:3] = bytes(self.device.axis_codes)
        elif code == 0x84006A34:
            for i, key in enumerate(self.device.button_codes):
                value[i] = key
        elif code == 0x81006A13:
            value[: len(self.device.name)] = self.device.name.encode()
        else:
            raise AssertionError(code)

    def register(self, fd, flags):
        pass

    def unregister(self, fd):
        pass

    def poll(self, timeout):
        self.clock.advance(0.001)
        return [(17, self.flags)] if self.chunks or self.flags != select.POLLIN else []

    def read(self, fd, count):
        self.read_sizes.append(count)
        value = self.chunks.pop(0)
        if isinstance(value, BaseException):
            raise value
        return value

    def source(self, profile=None):
        profile = profile or load_profile(CONFIG / "beitong_unverified.yaml")
        return NativeJoydev(
            profile,
            clock=self.clock,
            discovery=lambda _: self.device,
            opener=self.open,
            reader=self.read,
            ioctl=self.ioctl,
            poll_factory=lambda: self,
            closer=self.close,
        )

    def events(self, *events):
        self.chunks.append(b"".join(struct.pack("=IhBB", *event) for event in events))

    def baseline(self, stamp=1000):
        self.events(
            (stamp, 0, 0x82, 0),
            (stamp, 0, 0x82, 1),
            (stamp, -32768, 0x82, 2),
            (stamp, 0, 0x81, 0),
            (stamp, 0, 0x81, 1),
        )


def test_jt01_readonly_flags_init_baseline_no_press_and_poll_observation():
    import os

    kernel = Kernel()
    source = kernel.source()
    source.open()
    assert kernel.open_flags == [os.O_RDONLY | os.O_NONBLOCK | os.O_CLOEXEC]
    kernel.baseline()
    frames = source.poll()
    assert len(frames) == 5 and frames[-1].initialized
    assert all(f.event_seq == 0 and not f.changed_axes and not f.changed_buttons for f in frames)
    assert frames[-1].raw_event_ns == 0
    again = source.poll()[0]
    assert again.observation_seq > frames[-1].observation_seq
    assert again.raw_event_ns == frames[-1].raw_event_ns
    assert source.health().link_evidence == "UNKNOWN"
    heartbeat = source.health().reader_heartbeat_ns
    kernel.clock.advance(0.05)
    assert source.health().reader_heartbeat_ns == heartbeat
    source.close()
    assert kernel.closed == [17]


def test_jt01_b_press_release_in_one_read_are_preserved():
    kernel = Kernel()
    source = kernel.source()
    source.open()
    kernel.baseline()
    source.poll()
    kernel.events((1001, 1, 1, 1), (1001, 0, 1, 1))
    frames = source.poll()
    assert [f.buttons[1] for f in frames] == [1, 0]
    assert [f.event_seq for f in frames] == [1, 2]


def test_jt01_old_queue_never_gets_received_time_as_new_event():
    kernel = Kernel()
    source = kernel.source()
    source.open()
    kernel.baseline()
    source.poll()
    kernel.clock.advance(0.08)
    kernel.events((1001, 20000, 2, 0))
    frame = source.poll()[0]
    assert frame.observed_ns - frame.raw_event_ns >= 90_000_000
    kernel.clock.advance(0.08)
    kernel.events((1002, 20001, 2, 0))
    next_frame = source.poll()[0]
    assert next_frame.raw_event_ns - frame.raw_event_ns == 1_000_000
    assert next_frame.observed_ns - next_frame.raw_event_ns > 150_000_000


def test_jt01_kernel_u32_timestamp_wrap_is_continuous():
    kernel = Kernel()
    source = kernel.source()
    source.open()
    kernel.baseline(0xFFFFFFFE)
    source.poll()
    kernel.clock.advance(0.005)
    kernel.events((1, 20000, 2, 0))
    frame = source.poll()[0]
    assert frame.initialized and frame.raw_event_ns > 0
    assert source.health().connected


def test_jt01_reader_gap_refuses_old_queue_requires_reopen():
    kernel = Kernel()
    source = kernel.source()
    source.open()
    kernel.baseline()
    source.poll()
    epoch = source.health().input_epoch
    kernel.clock.advance(0.11)
    kernel.events((1001, 20000, 2, 0))
    assert source.poll() == ()
    assert source.health().reason == "READER_STALLED_REOPEN_REQUIRED"
    source.close()
    kernel.chunks.clear()
    source.open()
    assert source.health().input_epoch != epoch
    assert not source.health().initialized


def test_jt01_repeated_init_means_overflow_or_resync_new_epoch():
    kernel = Kernel()
    source = kernel.source()
    source.open()
    kernel.baseline()
    source.poll()
    epoch = source.health().input_epoch
    kernel.baseline(1001)
    frames = source.poll()
    assert frames[0].resync and frames[0].input_epoch != epoch
    assert not frames[0].initialized
    assert frames[-1].initialized and frames[-1].event_seq == 0


@pytest.mark.parametrize(
    "event,reason",
    [
        ((1000, 1, 2, 0), "MISSING_INIT_BASELINE"),
        ((1000, 1, 3, 0), "INVALID_EVENT_TYPE"),
        ((1000, 1, 2, 9), "INVALID_EVENT_VALUE"),
        ((1000, 2, 1, 0), "INVALID_EVENT_VALUE"),
    ],
)
def test_jt01_malformed_events_fail_closed(event, reason):
    kernel = Kernel()
    source = kernel.source()
    source.open()
    kernel.events(event)
    assert source.poll() == ()
    assert source.health().reason == reason


@pytest.mark.parametrize(
    "disconnect", [b"", OSError(errno.ENODEV, "gone"), OSError(errno.EIO, "gone")]
)
def test_jt01_disconnect_detected_without_autorepeat(disconnect):
    kernel = Kernel()
    source = kernel.source()
    source.open()
    kernel.chunks.append(disconnect)
    assert source.poll() == ()
    assert source.health().reason == "DISCONNECTED"
    assert not source.health().connected


def test_jt01_hup_and_future_timestamp_and_work_limits():
    kernel = Kernel()
    source = kernel.source()
    source.open()
    kernel.flags = select.POLLHUP
    assert source.poll() == () and not source.health().connected
    source.close()
    kernel.flags = select.POLLIN
    source.open()
    kernel.baseline()
    source.poll()
    kernel.events((999999, 1, 2, 0))
    assert source.poll() == ()
    assert source.health().reason == "EVENT_TIME_DISCONTINUITY"
    assert max(kernel.read_sizes) <= source.profile.max_poll_bytes
    with pytest.raises(ValueError):
        # Reopen for the bounded API check; closed readers return no data.
        source.close()
        source.open()
        source.poll(1000)


def test_jt01_partial_record_is_buffered_without_fabricating_event():
    kernel = Kernel()
    source = kernel.source()
    source.open()
    event = struct.pack("=IhBB", 1000, 0, 0x82, 0)
    kernel.chunks = [event[:3], event[3:]]
    first = source.poll()[0]
    assert first.axes[0] is None and not first.initialized
    assert source.poll()[0].axes[0] == 0


def test_identity_discovery_refuses_ambiguity_and_metadata_does_not_qualify(tmp_path):
    selector = load_profile(CONFIG / "beitong_unverified.yaml").device
    kernel = Kernel()
    for name in ("js0", "js1"):
        (tmp_path / name).mkdir()
    matched = replace(
        kernel.device,
        name=selector.name,
        axis_codes=selector.axis_codes,
        button_codes=selector.button_codes,
    )
    with pytest.raises(InputError, match="No unique"):
        discover(selector, sysfs_root=tmp_path, inspect=lambda *a, **kw: matched)
    (tmp_path / "js1").rmdir()
    assert discover(selector, sysfs_root=tmp_path, inspect=lambda *a, **kw: matched) == matched


def test_inspect_uses_ioctl_and_sysfs_no_event_read(tmp_path):
    kernel = Kernel()
    base = tmp_path / "js7/device"
    (base / "id").mkdir(parents=True)
    for name, value in [
        ("vendor", "20bc"),
        ("product", "506d"),
        ("bustype", "0003"),
        ("version", "0001"),
    ]:
        (base / "id" / name).write_text(value)
    device = inspect_device(
        "/dev/input/js7",
        opener=kernel.open,
        ioctl=kernel.ioctl,
        closer=kernel.close,
        sysfs_root=tmp_path,
        device_mode="explicit",
    )
    assert device.vendor == 0x20BC and device.product == 0x506D
    assert device.axis_codes == (0, 1, 2) and device.device_mode == "explicit"
    assert kernel.read_sizes == [] and kernel.closed == [17]


def test_jt08_script_open_is_idle_start_explicit_end_and_new_epoch(tmp_path):
    path = tmp_path / "script.json"
    path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "duration_s": 1,
                "events": [{"at_s": 0.1, "axes": {"0": 20000, "2": 32767}}],
            }
        )
    )
    profile = load_profile(CONFIG / "synthetic.yaml")
    clock = Clock()
    source = ScriptedInput(profile, script_file=path, clock=clock)
    source.open()
    assert source.poll() == () and source.health().reason == "SCRIPT_PAUSED"
    epoch = source.health().input_epoch
    source.start()
    assert source.health().input_epoch != epoch
    neutral = source.poll()[0]
    assert neutral.axes[0] == 0 and neutral.axes[2] == -32768
    clock.advance(0.1)
    moved = source.poll()[0]
    assert moved.axes[0] == 20000 and moved.synthetic
    clock.advance(1)
    assert source.poll() == () and source.health().reason == "SCRIPT_FINISHED"
    old = source.health().input_epoch
    source.start()
    assert source.health().input_epoch != old
    assert source.poll()[0].axes[0] == 0


def test_jt08_script_freeze_disconnect_reconnect_and_hardware_refusal():
    profile = load_profile(CONFIG / "synthetic.yaml")
    clock = Clock()
    source = ScriptedInput(profile, clock=clock)
    source.open()
    source.start()
    source.poll()
    heartbeat = source.health().reader_heartbeat_ns
    source.freeze()
    clock.advance(0.2)
    assert source.poll() == () and source.health().reader_heartbeat_ns == heartbeat
    source.freeze(False)
    assert source.health().reason == "SCRIPT_PAUSED"
    source.disconnect()
    assert not source.health().connected
    source.reconnect()
    assert source.health().reason == "SCRIPT_PAUSED"
    with pytest.raises(InputError, match="SCRIPT_REQUIRES_SIMULATION"):
        ScriptedInput(profile, backend="hardware")


@pytest.mark.parametrize(
    "event",
    [
        {"at_s": float("nan")},
        {"at_s": 0, "axes": {"0": True}},
        {"at_s": 0, "action": "exec"},
        {"at_s": 0, "buttons": {"0": 2}},
    ],
)
def test_jt08_script_is_bounded_validated_data_not_code(tmp_path, event):
    path = tmp_path / "bad.json"
    path.write_text(json.dumps({"schema_version": 1, "duration_s": 1, "events": [event]}))
    with pytest.raises(ValueError):
        load_script(path)


def test_jt01_identity_changed_between_discovery_and_open_rejected():
    kernel = Kernel()
    source = kernel.source()
    original = source.ioctl

    def changed_name(fd, code, value, mutate=True):
        original(fd, code, value, mutate)
        if code == 0x81006A13:
            value[:7] = b"changed"

    source.ioctl = changed_name
    with pytest.raises(InputError, match="IDENTITY_CHANGED_DURING_OPEN"):
        source.open()
    assert kernel.closed == [17]


def test_jt01_permission_denied_and_eagain_are_distinct():
    kernel = Kernel()
    source = kernel.source()

    def denied(path, flags):
        raise PermissionError(errno.EACCES, "test permission denied")

    source.opener = denied
    with pytest.raises(PermissionError):
        source.open()
    source = kernel.source()
    source.open()
    kernel.baseline()
    source.poll()
    kernel.chunks.append(BlockingIOError(errno.EAGAIN, "not ready"))
    frames = source.poll()
    assert frames[-1].initialized and source.health().connected
    assert frames[-1].event_seq == 0
