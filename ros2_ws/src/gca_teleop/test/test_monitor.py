"""Standalone input diagnosis: real pipeline gates, no ROS or actuator imports."""

import json
import selectors
import signal
import subprocess
import sys
import threading
from dataclasses import replace
from pathlib import Path

import pytest
from gca_teleop.config import TeleopConfig
from gca_teleop.model import InputError
from gca_teleop.monitor import InputObserver, TerminalLog, format_record, main, run_monitor
from test_input_pipeline import Frames, physical_evidence, synthetic

ROOT = Path(__file__).resolve().parents[4]


def observe(observer, fixture, **changes):
    frame = fixture.frame(**changes)
    return observer.process(frame, fixture.health(), fixture.now)


def test_monitor_raw_mode_never_guesses_mapping_or_calls_an_intent_motion():
    observer = InputObserver(TeleopConfig())
    fixture = Frames(real=True)
    first = observe(observer, fixture, real_event=False)
    assert first["sample_kind"] == "INIT_BASELINE"
    assert first["raw"]["event_seq"] == 0
    assert first["raw_event_age_ms"] is None
    held = observe(observer, fixture, axes={0: 32767, 2: 32767})
    assert held["sample_kind"] == "CHANGE"
    assert held["mapped_from_profile"] is None
    assert held["decision"]["gate"] == "RAW_UNMAPPED"
    assert held["decision"]["action"] == "NONE"
    assert held["motion_authorized"] is False
    assert "RAW_UNMAPPED" in format_record(held)


def test_monitor_shared_pipeline_expires_static_hold_but_keeps_raw_visible(tmp_path):
    profile, _, _ = physical_evidence(tmp_path)
    observer = InputObserver(profile)
    fixture = Frames(real=True)
    fixture.axes[2] = -32000
    assert observe(observer, fixture, real_event=False)["decision"]["gate"] == "READY"
    active = observe(observer, fixture, axes={1: -20000, 2: 32767})
    assert active["decision"]["action"] == "JOG"
    assert active["decision"]["linear_mm_s"] > 0
    assert active["decision"]["rotation_deg_s"] == 0
    deadline = active["decision"]["quality"]["valid_until_ns"]
    right = observe(observer, fixture, axes={3: 10000})
    assert right["reserved"]["axes"][3] == 10000
    assert right["decision"]["quality"]["valid_until_ns"] == deadline
    expired = observe(observer, fixture, advance=0.16, real_event=False)
    assert expired["decision"]["action"] == "STOP"
    assert expired["decision"]["reason"] == "GCA_INPUT_EXPIRED"
    assert expired["raw"]["axes"][1] == -20000
    assert expired["mapped_from_profile"]["ly"] > 0
    assert expired["decision"]["linear_mm_s"] == 0
    assert expired["gca_event_age_ms"] > 150
    assert expired["health"]["link_evidence"] == "UNKNOWN"
    assert observe(observer, fixture, axes={1: -21000})["decision"]["action"] != "JOG"
    observe(observer, fixture, axes={1: 0, 2: -32000})
    assert observe(observer, fixture, axes={1: -20000, 2: 32767})["decision"]["action"] == "JOG"


def test_monitor_b_stop_and_identity_mismatch_preserve_production_gate(tmp_path):
    profile, _, _ = physical_evidence(tmp_path)
    observer = InputObserver(profile)
    fixture = Frames(real=True)
    fixture.axes[2] = -32000
    observe(observer, fixture, real_event=False)
    observe(observer, fixture, axes={0: 20000, 2: 32767})
    stopped = observe(observer, fixture, buttons={1: 1})
    assert stopped["decision"]["reason"] == "B_STOP"
    assert stopped["decision"]["rotation_deg_s"] == 0
    assert observe(observer, fixture, buttons={1: 0})["decision"]["action"] != "JOG"
    fixture.epoch = "new-device-epoch"
    fixture.identity = replace(fixture.identity, kernel="different-kernel")
    mismatch = observe(observer, fixture, axes={0: 0, 2: -32000})
    assert mismatch["decision"]["reason"] == "MAPPING_EVIDENCE_MISMATCH"
    assert mismatch["motion_authorized"] is False
    assert mismatch["mapped_from_profile"] is not None  # interpretation, not qualification
    assert mismatch["gca_event_age_ms"] is None


def test_monitor_reopened_held_init_requires_release_center_and_new_lt_edge(tmp_path):
    profile, _, _ = physical_evidence(tmp_path)
    observer = InputObserver(profile)
    fixture = Frames(real=True)
    fixture.axes[2] = -32000
    observe(observer, fixture, real_event=False)
    assert observe(observer, fixture, axes={0: 20000, 2: 32767})["decision"]["action"] == "JOG"
    fixture.epoch, fixture.seq, fixture.raw = "reopened-held", 0, 0
    baseline = observe(observer, fixture, real_event=False)
    assert baseline["sample_kind"] == "INIT_BASELINE"
    assert baseline["raw"]["event_seq"] == 0
    assert baseline["decision"]["action"] != "JOG"
    assert observe(observer, fixture, axes={0: 21000})["decision"]["action"] != "JOG"
    neutral = observe(observer, fixture, axes={0: 0, 2: -32000})
    assert neutral["decision"]["gate"] == "READY"
    assert observe(observer, fixture, axes={0: 20000, 2: 32767})["decision"]["action"] == "JOG"


class CollectLog:
    error = ""
    dropped = 0

    def __init__(self):
        self.records = []
        self.written = 0

    def submit(self, record):
        self.records.append(record)
        self.written += 1
        return True


class IdleReader:
    def __init__(self, fixture):
        self.fixture = fixture
        self.closed = False

    def open(self):
        return self.fixture.identity

    def poll(self, timeout):
        return (self.fixture.frame(advance=timeout, real_event=False),)

    def health(self):
        return self.fixture.health()

    def close(self):
        self.closed = True


def test_monitor_idle_duration_samples_but_does_not_flood_terminal():
    fixture = Frames(real=True)
    reader, log = IdleReader(fixture), CollectLog()
    report = run_monitor(
        TeleopConfig(),
        log,
        duration_s=2.1,
        reader_factory=lambda profile: reader,
        clock=lambda: fixture.now,
    )
    assert reader.closed
    assert report["errors"] == []
    assert report["changed_frames"] == 0
    assert report["baseline_frames"] == 1
    samples = [r for r in log.records if r["kind"] == "observation"]
    assert 2 <= len(samples) <= 4
    assert fixture.obseq >= 100


def test_monitor_keyboard_interrupt_closes_reader():
    fixture = Frames(real=True)
    reader = IdleReader(fixture)

    def interrupt(timeout):
        raise KeyboardInterrupt

    reader.poll = interrupt
    report = run_monitor(
        TeleopConfig(),
        CollectLog(),
        reader_factory=lambda profile: reader,
        clock=lambda: fixture.now,
    )
    assert report["interrupted"] and reader.closed
    assert report["errors"] == []


def test_monitor_reconnect_wait_reopens_and_marks_new_baseline():
    fixture = Frames(real=True)
    first = IdleReader(fixture)
    second = IdleReader(fixture)
    missing = IdleReader(fixture)
    still_missing = IdleReader(fixture)
    original = first.poll
    polls = 0

    def disconnect(timeout):
        nonlocal polls
        polls += 1
        result = original(timeout)
        if polls == 2:
            first.health = lambda: fixture.health(
                connected=False, initialized=False, reason="DISCONNECTED"
            )
        return result

    def absent():
        raise InputError("INPUT_NOT_FOUND")

    first.poll = disconnect
    missing.open = absent
    still_missing.open = absent
    candidates = iter([first, missing, still_missing, second])

    def factory(profile):
        reader = next(candidates)
        if reader is second:
            fixture.epoch = "reopened"
            fixture.seq = 0
        return reader

    def wait(seconds):
        fixture.now += int(seconds * 1e9)

    log = CollectLog()
    report = run_monitor(
        TeleopConfig(),
        log,
        duration_s=3.5,
        reader_factory=factory,
        clock=lambda: fixture.now,
        wait=wait,
    )
    assert first.closed and missing.closed and still_missing.closed and second.closed
    assert report["errors"] == []
    assert report["reopens"] == 2 and report["reader_failures"] == 1
    assert report["baseline_frames"] == 2
    assert any("RECONNECT_WAIT" in r.get("health", {}).get("reason", "") for r in log.records)


def test_terminal_queue_drops_instead_of_blocking_reader_and_close_is_bounded():
    entered, release = threading.Event(), threading.Event()

    def sink(data):
        entered.set()
        assert release.wait(5)

    log = TerminalLog(capacity=2, sink=sink)
    try:
        log.submit({"kind": "start"})
        assert entered.wait(2)
        for n in range(20):
            log.submit({"kind": "test", "number": n})
        assert log.queue.qsize() == 2
        assert log.dropped == 18
        assert not log.close(timeout_s=0.01)
    finally:
        release.set()
        assert log.close(timeout_s=2)


def test_terminal_final_summary_survives_saturation_and_reports_all_dropped_rows():
    entered, release = threading.Event(), threading.Event()
    written = []

    def sink(data):
        entered.set()
        assert release.wait(5)
        written.append(json.loads(data))

    log = TerminalLog(capacity=2, sink=sink, jsonl=True)
    try:
        log.submit({"kind": "start"})
        assert entered.wait(2)
        for n in range(20):
            log.submit({"kind": "test", "number": n})
        fixture = Frames(real=True)
        reader = IdleReader(fixture)
        summary = run_monitor(
            TeleopConfig(),
            log,
            duration_s=0.01,
            reader_factory=lambda profile: reader,
            clock=lambda: fixture.now,
        )
        assert reader.closed
        assert summary["log_dropped"] == log.dropped == 21
    finally:
        release.set()
        assert log.close(timeout_s=2)
    assert written[-1]["kind"] == "summary"
    assert written[-1]["log_dropped"] == 21
    assert len(written) == 3


def _child(code):
    prefix = (
        "import sys\n"
        f"sys.path[:0] = {[str(ROOT / 'ros2_ws/src/gca_core'), str(ROOT / 'ros2_ws/src/gca_teleop')]!r}\n"
    )
    return [sys.executable, "-I", "-c", prefix + code]


@pytest.mark.parametrize("interrupt", [False, True])
def test_real_blocked_stdout_pipe_duration_or_sigint_closes_reader_and_exits(interrupt):
    code = """
import os, time
from gca_teleop.config import TeleopConfig
from gca_teleop.model import InputHealth, InputIdentity
from gca_teleop.monitor import TerminalLog, run_monitor
log = TerminalLog(capacity=8, jsonl=True)
for index in range(8):
    log.submit({'kind': 'large', 'payload': 'x' * 65536})
class Reader:
    closed = False
    def open(self):
        return InputIdentity('fixture', 'fixture', 1, 1, 3, 1, (0,), (1,))
    def poll(self, timeout):
        if not hasattr(self, 'notified'):
            self.notified = True
            os.write(2, b'READY\\n')
        time.sleep(.01)
        return ()
    def health(self):
        return InputHealth(True, False, time.monotonic_ns(), 'fixture', 'epoch', reason='BASELINE_PENDING')
    def close(self):
        self.closed = True
reader = Reader()
summary = run_monitor(TeleopConfig(), log, duration_s=DURATION, reader_factory=lambda profile: reader)
assert reader.closed and not summary['errors']
assert summary['interrupted'] == INTERRUPT
assert not log.close(.05)
raise SystemExit(0)
"""
    code = code.replace("DURATION", "None" if interrupt else ".2").replace(
        "INTERRUPT", repr(interrupt)
    )
    process = subprocess.Popen(_child(code), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    selector = selectors.DefaultSelector()
    try:
        selector.register(process.stderr, selectors.EVENT_READ)
        assert selector.select(3), "child did not start polling"
        assert process.stderr.readline() == b"READY\n"
        if interrupt:
            process.send_signal(signal.SIGINT)
        # Deliberately do not consume stdout until the process has exited.
        assert process.wait(timeout=4) == 0
        output, error = process.communicate(timeout=1)
        assert output and not error
    finally:
        selector.close()
        if process.poll() is None:
            process.kill()
            process.communicate(timeout=2)


def test_module_and_help_work_when_ros_and_actuator_imports_are_forbidden():
    code = """
import importlib.abc
class DenyROS(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        blocked = ('rclpy', 'rcl_interfaces', 'ament_index_python', 'gca_ros',
                   'gca_teleop.node', 'gca_teleop.session', 'gca_teleop.management',
                   'gca.devices', 'gca.runtime', 'gca_system_interfaces')
        if any(fullname == name or fullname.startswith(name + '.') for name in blocked):
            raise AssertionError('forbidden import: ' + fullname)
sys.meta_path.insert(0, DenyROS())
from gca_teleop.monitor import InputObserver, main
from gca_teleop.config import TeleopConfig
assert InputObserver(TeleopConfig()).pipeline is None
main(['--help'])
"""
    result = subprocess.run(_child(code), capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
    assert "--profile" in result.stdout and "--duration" in result.stdout


def test_monitor_rejects_scripted_profile_before_opening_input(tmp_path, capsys):
    from dataclasses import asdict

    path = tmp_path / "script.json"
    path.write_text(json.dumps(asdict(synthetic())))
    assert main(["--profile", str(path), "--duration", "0.01"]) == 2
    assert "not scripted input" in capsys.readouterr().err


@pytest.mark.parametrize("value", ["nan", "inf", "0", "-1", "86401"])
def test_monitor_rejects_unbounded_or_invalid_duration(value):
    with pytest.raises(SystemExit) as exc:
        main(["--duration", value])
    assert exc.value.code == 2
