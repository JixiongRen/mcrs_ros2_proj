"""ROS-free, read-only terminal observation of the production input pipeline.

The fixed OFFLINE_DIAGNOSTIC context exercises only input decisions. It is not
an ARM operation, a session, an execution permit or an actuator connection.
Raw cached values remain visible after a decision expires. Neither a successful
read nor this monitor's heartbeat establishes that a wireless link is alive.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import asdict
from math import isfinite
from queue import Empty, Full, Queue
from threading import Event, Lock, Thread
from time import monotonic_ns, sleep

from .config import TeleopConfig, load_profile
from .input.joydev import NativeJoydev
from .mapping import map_input
from .model import InputError, InputHealth, Intent
from .state_machine import TeleopPipeline

OFFLINE_CONTEXT = {
    "armed": True,
    "ready": True,
    "reference_id": "OFFLINE_DIAGNOSTIC_NO_DEVICE_REFERENCE",
    "session_key": "OFFLINE_DIAGNOSTIC_NO_SESSION",
    "hardware": False,
}


class InputObserver:
    """Reuse mapping and gates without constructing any ROS or session object."""

    def __init__(self, profile):
        if profile.source != "joydev":
            raise ValueError("The read-only monitor requires a joydev profile")
        self.profile = profile
        # Validate potentially large evidence before opening the native reader.
        self.pipeline = TeleopPipeline(profile) if profile.mapping.complete else None
        self.frame = None
        self.quality = None
        self._baseline_epoch = ""
        self._epoch = ""

    def process(self, frame, health, now_ns):
        if frame.input_epoch != self._epoch:
            self.quality = None
            self._epoch = frame.input_epoch
        self.frame = frame
        if frame.resync:
            kind = "RESYNC"
        elif frame.initialized and self._baseline_epoch != frame.input_epoch:
            self._baseline_epoch = frame.input_epoch
            kind = "INIT_BASELINE"
        elif frame.changed_axes or frame.changed_buttons:
            kind = "CHANGE"
        else:
            kind = "CACHE"
        intent = (
            self.pipeline.process(frame, health, now_ns, **OFFLINE_CONTEXT)
            if self.pipeline
            else Intent(gate="RAW_UNMAPPED", reason="NO_VERIFIED_MAPPING")
        )
        return self._snapshot(health, now_ns, intent, kind)

    def tick(self, health, now_ns):
        intent = (
            self.pipeline.tick(health, now_ns, **OFFLINE_CONTEXT)
            if self.pipeline
            else Intent(gate="RAW_UNMAPPED", reason=health.reason or "NO_VERIFIED_MAPPING")
        )
        return self._snapshot(health, now_ns, intent, "STATUS")

    def _snapshot(self, health, now_ns, intent, sample_kind):
        if intent.quality is not None:
            self.quality = intent.quality
        frame = self.frame
        mapped, mapping_error = None, ""
        reserved = {"axes": {}, "buttons": {}}
        if frame is not None and self.profile.mapping.complete and frame.initialized:
            try:
                mapped = asdict(map_input(frame, self.profile))
                reserved = {
                    "axes": {i: frame.axes[i] for i in self.profile.mapping.reserved_axes},
                    "buttons": {i: frame.buttons[i] for i in self.profile.mapping.reserved_buttons},
                }
            except (InputError, IndexError, TypeError) as exc:
                mapping_error = str(exc)
        quality = self.quality
        raw = (
            {key: value for key, value in asdict(frame).items() if key != "identity"}
            if frame is not None
            else None
        )
        return {
            "kind": "observation",
            "monotonic_ns": now_ns,
            "sample_kind": sample_kind,
            "context": "OFFLINE_DIAGNOSTIC",
            "motion_authorized": False,
            "raw": raw,
            "mapped_from_profile": mapped,
            "mapping_error": mapping_error,
            "reserved": reserved,
            "decision": asdict(intent),
            "health": {key: value for key, value in asdict(health).items() if key != "identity"},
            "reader_age_ms": (now_ns - health.reader_heartbeat_ns) / 1e6,
            "raw_event_age_ms": (now_ns - frame.raw_event_ns) / 1e6
            if frame is not None and frame.event_seq > 0
            else None,
            "gca_event_age_ms": (now_ns - quality.event_ns) / 1e6 if quality else None,
            "decision_remaining_ms": (intent.quality.valid_until_ns - now_ns) / 1e6
            if intent.quality
            else None,
        }


def _age(value):
    return "-" if value is None else f"{value:.1f}"


def format_record(record):
    """Human-readable line; structured JSONL retains the complete input fields."""
    if record["kind"] != "observation":
        return record["kind"].upper() + " " + json.dumps(record, ensure_ascii=False)
    raw, mapped = record["raw"], record["mapped_from_profile"]
    decision, health = record["decision"], record["health"]
    if raw is None:
        values = "raw=NO_FRAME"
    else:
        values = (
            f"seq={raw['event_seq']} epoch={raw['input_epoch'][:8]} "
            f"kernel_ms={raw['kernel_event_ms']} "
            f"axes={raw['axes']} buttons={raw['buttons']} "
            f"changed={raw['changed_axes']}/{raw['changed_buttons']}"
        )
    interpretation = (
        "mapped=UNAVAILABLE"
        if mapped is None
        else f"LX={mapped['lx']:+.3f} LY={mapped['ly']:+.3f} "
        f"LT={mapped['lt']:.3f} B={int(mapped['b'])} reserved={record['reserved']}"
    )
    return (
        f"t={record['monotonic_ns'] / 1e9:.3f} {record['sample_kind']} {values} | "
        f"{interpretation} | gate={decision['gate']} decision={decision['action']} "
        f"LT_held={int(decision['lt_pressed'])} "
        f"v={decision['linear_mm_s']:+.3f}mm/s w={decision['rotation_deg_s']:+.3f}deg/s "
        f"reason={decision['reason'] or '-'} mapping_error={record['mapping_error'] or '-'} | "
        f"raw_age={_age(record['raw_event_age_ms'])}ms "
        f"gca_age={_age(record['gca_event_age_ms'])}ms "
        f"reader_age={_age(record['reader_age_ms'])}ms "
        f"reader={'OK' if health['connected'] and health['initialized'] and not health['reason'] else health['reason'] or 'BASELINE_PENDING'} "
        f"radio={health['link_evidence']} log_dropped={record.get('log_dropped', 0)}"
    )


class TerminalLog:
    """A bounded output queue: a slow terminal cannot stop input polling.

    The worker uses os.write on stdout's FD, never the TextIOWrapper lock. If a
    pipe is blocked on exit, the daemon can be abandoned after a bounded join
    without hanging Python's final stdout flush. No O_NONBLOCK flag is changed
    on the inherited stdout open-file description. A supplied sink supports
    controlled tests; it must not access the reader or a ROS runtime.
    """

    def __init__(self, *, jsonl=False, capacity=256, sink=None):
        if type(capacity) is not int or not 1 <= capacity <= 4096:
            raise ValueError("log capacity must be 1..4096")
        self.jsonl = jsonl
        self.queue = Queue(maxsize=capacity)
        self.dropped = 0
        self.written = 0
        self.error = ""
        self._closing = Event()
        self._submit_lock = Lock()
        if sink is None:
            fd = sys.stdout.fileno()

            def sink(data):
                remaining = memoryview(data)
                while remaining:
                    written = os.write(fd, remaining)
                    if written <= 0:
                        raise OSError("stdout write made no progress")
                    remaining = remaining[written:]

        self._sink = sink
        self._thread = Thread(target=self._run, name="joystick-terminal-log", daemon=True)
        self._thread.start()

    def submit(self, record):
        with self._submit_lock:
            if self._closing.is_set():
                return False
            item = dict(record, log_dropped=self.dropped)
            try:
                self.queue.put_nowait(item)
                return True
            except Full:
                if record.get("kind") != "summary":
                    self.dropped += 1
                    return False
                # Keep a final accounting record even when ordinary samples
                # saturated the queue. The writer may have consumed a slot
                # meanwhile; concurrent producers cannot fill it under this lock.
                try:
                    self.queue.get_nowait()
                except Empty:
                    pass
                else:
                    self.dropped += 1
                item["log_dropped"] = self.dropped
                self.queue.put_nowait(item)
                return True

    def _run(self):
        try:
            while not self._closing.is_set() or not self.queue.empty():
                try:
                    record = self.queue.get(timeout=0.05)
                except Empty:
                    continue
                text = (
                    json.dumps(record, ensure_ascii=False, separators=(",", ":"))
                    if self.jsonl
                    else format_record(record)
                )
                self._sink((text + "\n").encode("utf-8", "replace"))
                self.written += 1
        except Exception as exc:
            self.error = type(exc).__name__ + ":" + str(exc)
            self._closing.set()

    def close(self, timeout_s=1.0):
        self._closing.set()
        self._thread.join(timeout_s)
        return not self._thread.is_alive()


def _signature(record):
    health, decision = record["health"], record["decision"]
    return (
        health["connected"],
        health["initialized"],
        health["input_epoch"],
        health["reason"],
        decision["action"],
        decision["gate"],
        decision["reason"],
        decision["lt_pressed"],
        decision["b_pressed"],
        record["mapping_error"],
    )


def run_monitor(
    profile,
    log,
    *,
    duration_s=None,
    reader_factory=NativeJoydev,
    clock=monotonic_ns,
    wait=sleep,
):
    """Return a summary after duration/Ctrl-C; never create a control session."""
    observer = InputObserver(profile)
    started = clock()
    deadline = None if duration_s is None else started + int(duration_s * 1e9)
    reader = None
    next_open = started
    last_signature = None
    last_status_ns = 0
    summary = {
        "kind": "summary",
        "context": "OFFLINE_DIAGNOSTIC",
        "motion_authorized": False,
        "started_ns": started,
        "changed_frames": 0,
        "baseline_frames": 0,
        "reopens": 0,
        "reader_failures": 0,
        "errors": [],
        "interrupted": False,
    }
    health = InputHealth(False, False, started, "", "", reason="NOT_OPEN")
    try:
        while deadline is None or clock() < deadline:
            if log.error:
                raise OSError("TERMINAL_LOG_FAILED:" + log.error)
            now = clock()
            if reader is None and now >= next_open:
                candidate = reader_factory(profile)
                try:
                    identity = candidate.open()
                except Exception as exc:
                    candidate.close()
                    if not summary["reopens"] or not (
                        isinstance(exc, OSError)
                        or isinstance(exc, InputError)
                        and exc.code == "INPUT_NOT_FOUND"
                    ):
                        raise
                    health = InputHealth(
                        False, False, clock(), "", "", reason="RECONNECT_WAIT:" + str(exc)
                    )
                    next_open = clock() + 1_000_000_000
                else:
                    reader = candidate
                    summary["reopens"] += 1
                    log.submit(
                        {
                            "kind": "device_open",
                            "monotonic_ns": clock(),
                            "identity": asdict(identity),
                            "mode": profile.mode,
                        }
                    )
            frames = ()
            if reader is not None:
                frames = reader.poll(profile.poll_timeout_s)
                health = reader.health()
            records = []
            for frame in frames:
                record = observer.process(frame, health, clock())
                summary["changed_frames"] += bool(frame.changed_axes or frame.changed_buttons)
                summary["baseline_frames"] += record["sample_kind"] == "INIT_BASELINE"
                records.append(record)
            if not frames or not health.connected or health.reason:
                records.append(observer.tick(health, clock()))
            for record in records:
                signature = _signature(record)
                if (
                    record["sample_kind"] in ("CHANGE", "INIT_BASELINE", "RESYNC")
                    or signature != last_signature
                    or record["monotonic_ns"] - last_status_ns >= 1_000_000_000
                ):
                    log.submit(record)
                    last_status_ns = record["monotonic_ns"]
                last_signature = signature
            if reader is not None and not health.connected:
                summary["reader_failures"] += 1
                reader.close()
                reader = None
                next_open = clock() + 1_000_000_000
            if reader is None:
                wait(min(profile.poll_timeout_s, max(0.0, (next_open - clock()) / 1e9)))
    except KeyboardInterrupt:
        summary["interrupted"] = True
    except Exception as exc:
        summary["errors"].append(type(exc).__name__ + ":" + str(exc))
    finally:
        if reader is not None:
            reader.close()
        summary.update(
            finished_ns=clock(),
            log_dropped=log.dropped,
            log_written=log.written,
            last_reader_health={
                key: value for key, value in asdict(health).items() if key != "identity"
            },
        )
        log.submit(summary)
        # Forcing the final summary into a full queue can itself evict one row.
        summary["log_dropped"] = log.dropped
    return summary


def _duration(value):
    try:
        result = float(value)
    except ValueError:
        raise argparse.ArgumentTypeError("duration must be a positive finite number") from None
    if not isfinite(result) or not 0 < result <= 86400:
        raise argparse.ArgumentTypeError("duration must be in (0, 86400] seconds")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="只读手柄终端监视器：无需 ROS，不发送任何运动命令。"
    )
    parser.add_argument("--profile", help="已标定 joydev profile；省略时只显示原始输入")
    parser.add_argument("--duration", type=_duration, help="运行秒数；省略则持续到 Ctrl-C")
    parser.add_argument("--jsonl", action="store_true", help="向终端输出逐行 JSON")
    args = parser.parse_args(argv)
    try:
        profile = load_profile(args.profile) if args.profile else TeleopConfig()
        if profile.source != "joydev":
            raise ValueError("The monitor only accepts physical joydev input, not scripted input")
        log = TerminalLog(jsonl=args.jsonl)
    except (OSError, ValueError, TypeError, ImportError) as exc:
        print("joystick monitor: " + str(exc), file=sys.stderr)
        return 2
    log.submit(
        {
            "kind": "monitor_start",
            "context": "OFFLINE_DIAGNOSTIC",
            "motion_authorized": False,
            "profile": args.profile,
            "mode": profile.mode,
            "mapping": "PROFILE_INTERPRETATION_WITH_PIPELINE_EVIDENCE_CHECK"
            if profile.mapping.complete
            else "RAW_UNMAPPED",
            "notice": (
                "只读观察：raw/映射值与门控判定分别显示；JOG/STOP仅是输入判定，不是执行器命令。"
                "reader健康不等于无线在线。event_age静态保持会过期；Ctrl-C退出。"
            ),
        }
    )
    try:
        summary = run_monitor(profile, log, duration_s=args.duration)
    except (OSError, ValueError, TypeError) as exc:
        summary = {"errors": [str(exc)]}
    drained = log.close()
    if not drained:
        print("joystick monitor: terminal output blocked; input reader closed", file=sys.stderr)
    if summary["errors"] or log.error:
        print("joystick monitor: " + "; ".join(summary["errors"] + [log.error]), file=sys.stderr)
    return 2 if summary["errors"] or log.error or not drained else 0


if __name__ == "__main__":
    raise SystemExit(main())
