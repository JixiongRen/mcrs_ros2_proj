#!/usr/bin/env python3
"""Installed joystick pipeline -> real DDS -> simulated actuators and plant.

Scripts enter the ordinary input port. Management enters the installed owner's
local IPC; this runner never sends Jog directly or reads private credentials.
The separate model report explicitly uses model time and independent truth.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib
import json
import math
import os
import subprocess
import tempfile
import threading
import time
import traceback
import xml.etree.ElementTree as ET
from dataclasses import replace
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from gca_system_interfaces.msg import GcaState, TeleopState
from gca_teleop.cli import request
from gca_teleop.config import load_profile
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from simulation_suite import stop_owned
from std_msgs.msg import String
from verify_software import source_manifest

SCENARIOS = (
    "linear",
    "rotation",
    "diagonal_reverse",
    "b_stop",
    "reader_freeze",
    "disconnect",
    "slow_arm",
    "control_freeze",
    "lt_release_restore",
)


def script_for(name):
    events = [
        {"at_s": 0.0, "axes": {"0": 0, "1": 0, "2": -32768}, "buttons": {"1": 0}},
        {"at_s": 0.4, "axes": {"2": 32767}},
    ]
    if name == "slow_arm":
        return {
            "schema_version": 1,
            "duration_s": 1.8,
            "events": [{"at_s": 0.0, "axes": {"2": -32768}}, {"at_s": 0.1, "buttons": {"1": 1}}],
        }
    motion = {"0": 0, "1": -32768}
    if name == "rotation":
        motion = {"0": 32767, "1": 0}
    if name in ("diagonal_reverse", "control_freeze", "lt_release_restore"):
        motion = {"0": 32767, "1": -32768}
    events.append({"at_s": 0.6, "axes": motion})
    # Pre-reserved right controls must have no GCA effect.
    events.extend(
        [
            {
                "at_s": 1.0,
                "axes": {"3": 32767, "4": -32768, "5": 32767},
                "buttons": {"5": 1, "9": 1},
            },
            {"at_s": 1.2, "axes": {"3": 0, "4": 0, "5": 0}, "buttons": {"5": 0, "9": 0}},
        ]
    )
    if name == "lt_release_restore":
        events.append({"at_s": 1.5, "axes": {"2": -32768}})
    elif name == "b_stop":
        events.extend(
            [
                {"at_s": 1.5, "buttons": {"1": 1}},
                {"at_s": 1.6, "buttons": {"1": 0}},
                {"at_s": 1.8, "axes": {"1": 0}},
                {"at_s": 2.0, "axes": {"1": -32768}},
                {"at_s": 2.8, "axes": {"1": 0, "2": -32768}},
                {"at_s": 3.1, "axes": {"2": 32767}},
                {"at_s": 3.2, "axes": {"1": -32768}},
            ]
        )
    elif name == "reader_freeze":
        events.append({"at_s": 1.5, "action": "freeze"})
    elif name == "disconnect":
        events.append({"at_s": 1.5, "action": "disconnect"})
    elif name == "diagonal_reverse":
        events.append({"at_s": 2.1, "axes": {"0": -32768, "1": 32767}})
    events.extend([{"at_s": 4.0, "axes": {"0": 0, "1": 0}}, {"at_s": 4.8, "axes": {"2": -32768}}])
    return {"schema_version": 1, "duration_s": 5.5, "events": events}


def wait_for(predicate, timeout_s, label, process=None):
    deadline = time.monotonic() + timeout_s
    last_error = None
    while time.monotonic() < deadline:
        if process is not None and process.poll() is not None:
            raise RuntimeError(f"launch exited {process.returncode}: {label}")
        try:
            result = predicate()
            if result:
                return result
        except (OSError, RuntimeError, ValueError) as exc:
            last_error = str(exc)
        time.sleep(0.02)
    raise TimeoutError(f"{label}: {last_error or 'condition unmet'}")


class Observer:
    def __init__(self, domain):
        self.context = Context()
        rclpy.init(args=[], context=self.context, domain_id=domain)
        self.node = Node("teleop_acceptance_observer", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.lock = threading.Lock()
        self.rows, self.teleop, self.phases, self.evidence = [], [], [], []
        self.errors = []
        qos = QoSProfile(
            depth=16,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.node.create_subscription(
            GcaState, "/system/state", lambda m: self.append(self.rows, m), qos
        )
        self.node.create_subscription(
            TeleopState, "/teleop/state", lambda m: self.append(self.teleop, m), 10
        )
        self.node.create_subscription(String, "/gca/arm_phase", self.phase, qos)
        self.node.create_subscription(DiagnosticArray, "/diagnostics", self.diagnostic, 20)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def append(self, target, message):
        row = dict(message_to_ordereddict(message), received_monotonic_ns=time.monotonic_ns())
        with self.lock:
            if len(target) >= 20_000:
                self.errors.append("OBSERVER_CAPACITY_EXCEEDED")
                return
            target.append(row)

    def phase(self, message):
        try:
            row = dict(json.loads(message.data), received_monotonic_ns=time.monotonic_ns())
        except (ValueError, TypeError) as exc:
            with self.lock:
                self.errors.append("ARM_PHASE_PARSE:" + str(exc))
            return
        with self.lock:
            self.phases.append(row)
            if len(self.phases) > 256:
                del self.phases[0]

    def diagnostic(self, message):
        for status in message.status:
            if status.name != "gca_control":
                continue
            for value in status.values:
                if value.key != "simulation_io_evidence":
                    continue
                try:
                    row = dict(json.loads(value.value), received_monotonic_ns=time.monotonic_ns())
                    with self.lock:
                        if len(self.evidence) >= 20_000:
                            self.errors.append("IO_OBSERVER_CAPACITY_EXCEEDED")
                        else:
                            self.evidence.append(row)
                except (ValueError, TypeError) as exc:
                    with self.lock:
                        self.errors.append("IO_EVIDENCE_PARSE:" + str(exc))

    def io_snapshot(self):
        with self.lock:
            return list(self.evidence)

    def snapshot(self):
        with self.lock:
            return list(self.rows), list(self.teleop), list(self.phases)

    def close(self):
        self.executor.shutdown(timeout_sec=2)
        self.thread.join(2)
        self.node.destroy_node()
        self.context.shutdown()


def export(directory, observer, start_ns):
    rows, teleop, phases = observer.snapshot()
    for filename, data in (
        ("states.jsonl", rows),
        ("teleop.jsonl", teleop),
        ("arm_phases.jsonl", phases),
        ("simulation_io.jsonl", observer.io_snapshot()),
    ):
        (directory / filename).write_text("".join(json.dumps(row) + "\n" for row in data))
    fields = [
        "time_s",
        "linear_position_mm",
        "measured_linear_mm_s",
        "applied_linear_mm_s",
        "rotation_estimated_deg",
        "confirmed_pulses",
        "reserved_pulses",
        "stop_serial",
        "input_valid_until_ns",
    ]
    plotted = []
    for row in rows:
        plotted.append(
            [
                (row["received_monotonic_ns"] - start_ns) / 1e9,
                row["linear_position_m"] * 1000,
                row["linear_velocity_m_s"] * 1000,
                row["applied_velocity_m_s"] * 1000,
                math.degrees(row["rotation_estimated_rad"]),
                row["confirmed_pulses"],
                row["reserved_pulses"],
                row["stop_serial"],
                row["input_valid_until_ns"],
            ]
        )
    with (directory / "states.csv").open("w") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        writer.writerows(plotted)
    independent = [r for r in observer.io_snapshot() if r.get("available")]
    io_fields = sorted({key for row in observer.io_snapshot() for key in row})
    if io_fields:
        with (directory / "simulation_io.csv").open("w") as stream:
            writer = csv.DictWriter(stream, fieldnames=io_fields)
            writer.writeheader()
            writer.writerows(observer.io_snapshot())
    if plotted:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        xs = [r[0] for r in plotted]
        axes[0].plot(xs, [r[2] for r in plotted], label="external encoder velocity")
        axes[0].plot(xs, [r[3] for r in plotted], label="cached servo receipt", alpha=0.7)
        if independent:
            io_x = [(r["sampled_ns"] - start_ns) / 1e9 for r in independent]
            axes[0].plot(
                io_x,
                [r["plant_velocity_mm_s"] for r in independent],
                label="independent simulated plant",
                alpha=0.7,
            )
            axes[0].plot(
                io_x,
                [r["accepted_velocity_mm_s"] for r in independent],
                label="independent simulated I/O",
                linestyle="--",
            )
            axes[1].plot(
                io_x,
                [r["plant_angle_deg"] for r in independent],
                label="independent mechanical truth",
                linestyle="--",
            )
            axes[1].legend()
        axes[0].set_ylabel("mm/s")
        axes[0].legend()
        axes[1].plot(xs, [r[4] for r in plotted])
        axes[1].set_ylabel("pulse estimate (deg)")
        axes[2].step(xs, [r[7] for r in plotted], where="post")
        axes[2].set_ylabel("STOP serial")
        axes[2].set_xlabel("host monotonic time from script start (s)")
        fig.tight_layout()
        for suffix in ("png", "svg"):
            fig.savefig(directory / ("curves." + suffix))
        plt.close(fig)
    return rows, teleop, phases


def installed_manifest():
    """Hash the files actually selected by ament/import, including both launch files."""
    files = {}
    for package, relative in (
        ("gca_teleop", "config/synthetic.yaml"),
        ("system_bringup", "config/simulation.yaml"),
        ("system_bringup", "config/teleop_acceptance.yaml"),
        ("system_bringup", "launch/teleop_simulation.launch.py"),
        ("system_bringup", "launch/simulation.launch.py"),
    ):
        files[package + "/" + relative] = Path(get_package_share_directory(package)) / relative
    for name in (
        "gca_teleop.node",
        "gca_teleop.session",
        "gca_teleop.dispatch",
        "gca_ros.node",
        "system_coordinator.node",
        "gca.runtime.control",
    ):
        files[name] = Path(importlib.import_module(name).__file__)
    return {
        name: {
            "path": str(path.resolve()),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "size": path.stat().st_size,
        }
        for name, path in files.items()
    }


def declared_budgets(criteria):
    """All waits are selected before motion, rather than widened after failure."""
    ros = criteria["ros"]
    return dict(
        startup_s=float(ros["startup_timeout_s"]),
        management_s=float(ros["management_timeout_s"]),
        stop_s=float(ros["stop_observation_timeout_s"]),
        post_stop_s=float(ros["post_stop_observation_s"]),
        readiness_s=6.0,
        reference_s=3.0,
        arm_barrier_s=3.0,
        cli_s=10.0,
        script_margin_s=0.3,
        arm_pause_max_s=1.0,
        recovery_motion_s=1.5,
    )


def load_jsonl_records(directory, *, core=False):
    paths = (
        (directory / "ros_logs").glob("gca-records-*/control-*.jsonl")
        if core
        else (directory / "input_records").glob("control-*.jsonl")
    )
    records = []
    for path in sorted(paths):
        if path.stat().st_size > 2_000_000 + 16_384:
            raise AssertionError("RECORDER_FILE_BOUND:" + str(path))
        for line in path.read_text().splitlines():
            row = json.loads(line)
            row["_file"] = str(path.relative_to(directory))
            records.append(row)
    return sorted(records, key=lambda row: row.get("monotonic_ns", row.get("time_ns", 0)))


def percentile(values, percent):
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * percent / 100.0
    low = math.floor(index)
    return ordered[low] + (ordered[math.ceil(index)] - ordered[low]) * (index - low)


def latency_stats(values):
    return dict(
        count=len(values),
        p50_ms=percentile(values, 50),
        p95_ms=percentile(values, 95),
        max_ms=max(values) if values else None,
    )


def stop_timelines(records, core, io, states, start_ns):
    """Keep source, host receipt, control and independent I/O observations distinct."""
    requests = [
        r for r in records if r.get("kind") == "stop_request" and r["monotonic_ns"] >= start_ns
    ]
    accepts = {r["sequence"]: r for r in records if r.get("kind") == "stop_accepted"}
    input_stops = [
        r
        for r in records
        if r.get("kind") == "input" and r.get("intent", {}).get("action") == "STOP"
    ]
    timelines = []
    for request_row in requests:
        sent = request_row["monotonic_ns"]
        accepted = accepts.get(request_row["sequence"])
        matching = [
            r
            for r in input_stops
            if r["intent"].get("reason") == request_row["reason"]
            and r.get("frame", {}).get("observed_ns", sent + 1) <= sent
        ]
        source = max(matching, key=lambda r: r["frame"]["observed_ns"], default=None)
        timeline = dict(
            reason=request_row["reason"],
            sequence=request_row["sequence"],
            generation=request_row["generation"],
            source_event_ns=(source["frame"].get("raw_event_ns") if source else None),
            reader_observed_ns=(source["frame"]["observed_ns"] if source else None),
            owner_request_ns=sent,
            owner_accepted_ns=(accepted["monotonic_ns"] if accepted else None),
            stop_serial=(accepted.get("stop_serial") if accepted else None),
            boot_id=(accepted.get("boot_id") if accepted else None),
        )
        if accepted and accepted.get("stop_serial"):
            serial, boot = accepted["stop_serial"], accepted["boot_id"]
            core_rows = [
                r
                for r in core
                if r.get("boot_id") == boot
                and r.get("stop_serial", -1) >= serial
                and r["time_ns"] >= sent
            ]
            observed = [
                r
                for r in states
                if r["boot_id"] == boot
                and r["stop_serial"] >= serial
                and r["received_monotonic_ns"] >= sent
            ]
            io_rows = [
                r
                for r in io
                if r.get("available")
                and r.get("stop_serial", -1) >= serial
                and r["sampled_ns"] >= sent
            ]
            confirmed = [r for r in observed if r["stop"]["progress"] == "CONFIRMED"]
            zero = [
                r for r in io_rows if abs(r["accepted_velocity_mm_s"]) < 1e-9 and r["pulse_idle"]
            ]
            timeline.update(
                control_first_record_ns=min((r["time_ns"] for r in core_rows), default=None),
                independent_io_first_observed_ns=min((r["sampled_ns"] for r in zero), default=None),
                confirmation_first_dds_observed_ns=min(
                    (r["received_monotonic_ns"] for r in confirmed), default=None
                ),
            )
        for field in (
            "owner_accepted_ns",
            "control_first_record_ns",
            "independent_io_first_observed_ns",
            "confirmation_first_dds_observed_ns",
        ):
            if timeline.get(field) is not None:
                timeline[field.removesuffix("_ns") + "_from_request_ms"] = (
                    timeline[field] - sent
                ) / 1e6
        timelines.append(timeline)
    summaries = {}
    for field in (
        "owner_accepted",
        "control_first_record",
        "independent_io_first_observed",
        "confirmation_first_dds_observed",
    ):
        key = field + "_from_request_ms"
        summaries[field] = latency_stats([r[key] for r in timelines if key in r])
    return dict(
        clock="host_monotonic_ns",
        interpretation="Observed upper bounds, not hard realtime or physical actuator latency",
        samples=timelines,
        summaries=summaries,
    )


def scenario_checks(name, rows, teleop, io, records, phases, start_ns, criteria, runtime, markers):
    """Pure acceptance analysis. Missing samples always fail, never vacuous all()."""
    checks = []

    def check(label, condition, details=None):
        checks.append(dict(name=label, passed=bool(condition), details=details))

    def relative(row):
        return (row["sampled_ns"] - start_ns) / 1e9

    def window(begin, end):
        return [r for r in available if begin <= relative(r) <= end]

    def zero_window(label, begin, end):
        chosen = window(begin, end)
        check(
            label,
            len(chosen) >= 2
            and all(abs(r["accepted_velocity_mm_s"]) < 1e-9 and r["pulse_idle"] for r in chosen),
            dict(start_s=begin, end_s=end, samples=len(chosen)),
        )

    available = [
        r
        for r in io
        if r.get("available")
        and r.get("evidence_kind") == "SIMULATED_ACCEPTED_IO_AND_PLANT"
        and r["sampled_ns"] >= start_ns
    ]
    inputs = [
        r
        for r in records
        if r.get("kind") == "input" and r.get("frame", {}).get("observed_ns", 0) >= start_ns
    ]
    publications = [
        r for r in records if r.get("kind") == "jog_published" and r["monotonic_ns"] >= start_ns
    ]
    pulse_deg = 360 * runtime["catheter_per_motor_ratio"] / runtime["pulses_per_motor_rev"]
    check("real_dds_state_observed", len(rows) >= 2)
    check("real_dds_teleop_observed", len(teleop) >= 2)
    check("independent_simulated_io_observed", len(available) >= 2)
    check(
        "bounded_pending_and_inflight",
        bool(teleop) and all(r["pending_count"] <= 1 and r["inflight_count"] <= 1 for r in teleop),
    )
    check(
        "formal_script_input_recorded",
        bool(inputs) and all(r["frame"]["synthetic"] for r in inputs),
    )
    quality_inputs = [
        r["intent"]["quality"] for r in inputs if r.get("intent", {}).get("action") == "JOG"
    ]
    if name != "slow_arm":
        check(
            "original_quality_deadline_on_every_jog",
            bool(quality_inputs)
            and all(
                q["source_kind"] == "SCRIPT_SAMPLE"
                and q["observation_ns"]
                < q["valid_until_ns"]
                <= q["observation_ns"] + round(criteria["ros"]["input_max_age_s"] * 1e9)
                for q in quality_inputs
            ),
        )
        check("formal_jog_dispatched", bool(publications))
    if name not in ("rotation", "slow_arm"):
        check(
            "accepted_forward_linear_output",
            any(r["accepted_velocity_mm_s"] > 0.1 for r in available),
        )
        check(
            "independent_plant_moved_forward",
            bool(available)
            and max(r["plant_position_mm"] for r in available)
            - min(r["plant_position_mm"] for r in available)
            > criteria["linear"]["quantization_counts"] * runtime["encoder_mm_per_count"],
        )
    if name in ("rotation", "diagonal_reverse", "control_freeze", "lt_release_restore"):
        check(
            "rotation_emitted_edges_move_plant",
            bool(available)
            and max(r["plant_angle_deg"] for r in available)
            - min(r["plant_angle_deg"] for r in available)
            > 2 * pulse_deg,
        )
    if name in ("rotation", "diagonal_reverse"):
        check(
            "existing_confirmed_rotation_smoke_threshold",
            any(abs(r["confirmed_pulses"]) > 20 for r in rows),
        )
    if name in ("linear", "b_stop", "reader_freeze", "disconnect"):
        check(
            "linear_only_has_no_rotation_output",
            bool(available) and max(abs(r["plant_angle_deg"]) for r in available) <= pulse_deg,
        )
    if name == "rotation":
        check(
            "rotation_only_has_no_linear_output",
            bool(available) and all(abs(r["accepted_velocity_mm_s"]) < 1e-9 for r in available),
        )
    if name in ("linear", "rotation"):
        chosen = window(
            0.6 + criteria["linear"]["settle_s"],
            0.6 + criteria["linear"]["settle_s"] + criteria["linear"]["window_s"],
        )
        check("steady_window_has_independent_samples", len(chosen) >= 3)
        if len(chosen) >= 3:
            elapsed = (chosen[-1]["plant_updated_ns"] - chosen[0]["plant_updated_ns"]) / 1e9
            if name == "linear":
                measured = (
                    chosen[-1]["plant_position_mm"] - chosen[0]["plant_position_mm"]
                ) / elapsed
                limit = max(
                    criteria["linear"]["relative_error"] * 2.0,
                    criteria["linear"]["quantization_counts"]
                    * runtime["encoder_mm_per_count"]
                    / elapsed,
                )
                check(
                    "linear_steady_velocity_matches_declared_criterion",
                    abs(measured - 2.0) <= limit,
                    dict(measured_mm_s=measured, target_mm_s=2.0, tolerance_mm_s=limit),
                )
            else:
                measured = (chosen[-1]["plant_angle_deg"] - chosen[0]["plant_angle_deg"]) / elapsed
                limit = max(
                    criteria["rotation"]["relative_rate_error"] * 30.0,
                    criteria["rotation"]["quantization_pulses"] * pulse_deg / elapsed,
                )
                check(
                    "rotation_steady_rate_matches_declared_criterion",
                    abs(measured - 30.0) <= limit,
                    dict(measured_deg_s=measured, target_deg_s=30.0, tolerance_deg_s=limit),
                )
        neutral = window(4.3, 5.3)
        check(
            "neutral_has_no_position_or_pulse_catchup",
            len(neutral) >= 3
            and max(r["plant_position_mm"] for r in neutral)
            - min(r["plant_position_mm"] for r in neutral)
            <= criteria["linear"]["neutral_drift_counts"] * runtime["encoder_mm_per_count"]
            and max(r["plant_angle_deg"] for r in neutral)
            - min(r["plant_angle_deg"] for r in neutral)
            <= pulse_deg,
        )
    if name not in ("slow_arm", "control_freeze"):
        expected_linear = 0.0 if name == "rotation" else 2.0
        expected_rotation = (
            30.0 if name in ("rotation", "diagonal_reverse", "lt_release_restore") else 0.0
        )
        reserved = [
            r
            for r in inputs
            if 1.04 < (r["frame"]["observed_ns"] - start_ns) / 1e9 < 1.18
            and r["intent"].get("action") == "JOG"
        ]
        check(
            "right_reserved_controls_do_not_change_gca_intent",
            bool(reserved)
            and all(
                abs(r["intent"]["linear_mm_s"] - expected_linear) < 1e-9
                and abs(r["intent"]["rotation_deg_s"] - expected_rotation) < 1e-9
                for r in reserved
            ),
        )
    if name == "diagonal_reverse":
        before, after = window(1.3, 1.9), window(2.7, 3.8)
        check(
            "both_axes_active_together",
            len(before) >= 2
            and any(r["accepted_velocity_mm_s"] > 0.5 for r in before)
            and before[-1]["plant_angle_deg"] - before[0]["plant_angle_deg"] > 2 * pulse_deg,
        )
        check(
            "both_axes_reverse_after_opposite_input",
            len(after) >= 2
            and any(r["accepted_velocity_mm_s"] < -0.5 for r in after)
            and after[-1]["plant_angle_deg"] - after[0]["plant_angle_deg"] < -2 * pulse_deg,
        )
    if name == "b_stop":
        zero_window("B_release_and_held_LT_cannot_resume", 2.1, 2.7)
        check(
            "B_lock_has_no_new_jog_publication",
            not any(2.1 < (r["monotonic_ns"] - start_ns) / 1e9 < 2.7 for r in publications),
        )
        check(
            "fresh_LT_cycle_resumes",
            any(3.5 < relative(r) < 4 and r["accepted_velocity_mm_s"] > 0.5 for r in available),
        )
        check(
            "B_stop_went_through_owner_service",
            any(r.get("kind") == "stop_request" and r.get("reason") == "B_STOP" for r in records),
        )
    if name in ("reader_freeze", "disconnect"):
        zero_window("input_failure_stops_without_cached_resume", 2.3, 5.0)
        reason = "READER_STALE" if name == "reader_freeze" else "DISCONNECTED"
        check(
            "input_failure_reason_reached_stop_service",
            any(r.get("kind") == "stop_request" and r.get("reason") == reason for r in records),
        )
        check(
            "failed_input_never_manufactures_new_jog",
            not any(2.3 < (r["monotonic_ns"] - start_ns) / 1e9 < 5.0 for r in publications),
        )
    if name == "slow_arm":
        start = markers.get("arm_started_ns", start_ns)
        waits = [
            r for r in phases if r.get("phase") == "WAITING" and r["received_monotonic_ns"] >= start
        ]
        check("real_arm_waiting_barrier_observed", bool(waits))
        check(
            "B_preempted_ARM_and_no_motion_ever_started",
            bool(available)
            and all(
                not r["servo_enabled"]
                and abs(r["accepted_velocity_mm_s"]) < 1e-9
                and r["pulse_idle"]
                for r in available
            )
            and not publications,
        )
        check("ARM_result_explicitly_failed", markers.get("arm_rejected") is True)
        check(
            "B_stop_received_while_ARM_waited",
            any(r.get("kind") == "stop_request" and r.get("reason") == "B_STOP" for r in records),
        )
        check(
            "late_ARM_never_reenabled_after_pause_clear",
            any(
                r["sampled_ns"]
                >= markers.get("arm_pause_cleared_ns", 2**64)
                + round(criteria["ros"]["post_stop_observation_s"] * 1e9)
                and not r["servo_enabled"]
                for r in available
            ),
        )
    if name == "control_freeze":
        begin, end = markers.get("freeze_started_ns", 0), markers.get("freeze_cleared_ns", 0)
        frozen = [
            r for r in available if begin <= r["sampled_ns"] <= end and r.get("control_paused")
        ]
        check(
            "control_frozen_while_independent_io_keeps_observing",
            len(frozen) >= 2
            and len({r["control_state_seq"] for r in frozen}) == 1
            and frozen[-1]["plant_updated_ns"] > frozen[0]["plant_updated_ns"],
        )
        if frozen:
            deadline = min(
                r["input_valid_until_ns"] for r in frozen if r["input_valid_until_ns"] > 0
            )
            io_budget_ns = round((runtime["io_period_s"] + runtime["io_timeout_s"]) * 1e9)
            expired = [r for r in frozen if r["sampled_ns"] >= deadline + io_budget_ns]
            check(
                "original_input_deadline_not_extended_by_live_source",
                len({r["input_valid_until_ns"] for r in frozen}) == 1
                and bool(expired)
                and all(
                    abs(r["accepted_velocity_mm_s"]) < 1e-9 and r["pulse_idle"] for r in expired
                ),
                dict(original_input_deadline_ns=deadline, io_transaction_budget_ns=io_budget_ns),
            )
            check(
                "finite_segment_and_servo_demand_fit_original_deadline",
                all(
                    (not r["demand_expires_ns"] or r["demand_expires_ns"] <= deadline)
                    and (
                        not r["pulse_active_expires_ns"] or r["pulse_active_expires_ns"] <= deadline
                    )
                    for r in frozen
                ),
            )
            live = [r for r in inputs if begin <= r["frame"]["observed_ns"] <= end]
            check(
                "reader_continues_real_synthetic_observations_during_control_freeze",
                len(live) >= 2
                and live[-1]["frame"]["observation_seq"] > live[0]["frame"]["observation_seq"],
            )
        else:
            check(
                "original_input_deadline_not_extended_by_live_source",
                False,
                "missing frozen I/O evidence",
            )
    if name == "lt_release_restore":
        recovery = markers.get("recovery_started_ns", 0)
        quiet_end = max(2.2, (recovery - start_ns) / 1e9 - 0.1)
        zero_window("LT_release_with_deflected_stick_stays_stopped", 2.1, quiet_end)
        check(
            "LT_release_reaches_formal_stop_path",
            any(
                r.get("kind") == "stop_request" and r.get("reason") == "DEADMAN_RELEASED"
                for r in records
            ),
        )
        check(
            "same_owner_reference_and_arm_explicitly_restored",
            markers.get("reference_changed") is True and markers.get("same_owner_restored") is True,
        )
        check(
            "new_neutral_LT_cycle_after_reference_really_moves",
            recovery > 0
            and any(
                r["sampled_ns"] >= recovery + 800_000_000 and r["accepted_velocity_mm_s"] > 0.5
                for r in available
            ),
        )
    return checks


# These are coverage declarations, not claims that this one runner proves every JT.
COVERAGE = {
    "JT01": "Input parsing/device identity: source unit/probe tests; physical NOT_RUN until captured",
    "JT02": "DDS linear/rotation plus mapping unit tests",
    "JT03": "DDS diagonal_reverse and reserved-control assertions",
    "JT04": "DDS b_stop/lt_release_restore plus hysteresis unit tests",
    "JT05": "Idle launch and slow_arm; reconnect/context gates in source tests",
    "JT06": "Physical EVENT_AGE static expiry in input/state-machine tests; SCRIPT_SAMPLE is distinct",
    "JT07": "Qualification mismatch/qualified-state fixtures in source tests; actual RF NOT_RUN",
    "JT08": "Quality/PROGRAMMED_JOG wire tests; boot/epoch replay in receiver tests",
    "JT09": "DDS occupancy observed; deterministic slow permit/source tests prove no starvation",
    "JT10": "DDS B path plus deterministic gate/I/O tests for every interleaving",
    "JT11": "DDS slow_arm WAITING→B plus alias/ACK/timeout unit regressions",
    "JT12": "DDS control_freeze with independent simulated I/O and original deadline",
    "JT13": "DDS linear steady criterion; independent model covers slip/noise/delay/saturation",
    "JT14": "DDS diagonal_reverse plus model and first-Jog/dt unit regressions",
    "JT15": "Independent model fine rotation and phase/rounding regressions",
    "JT16": "Shared profile/actual pigpio adapter offline tests plus independent model edge checks; hardware NOT_RUN",
    "JT17": "DDS neutral/no-catchup and lt_release_restore",
    "JT18": "Owned-process cleanup here; source/recorder/queue/signal tests in unified verification",
    "JT19": "Independent fixed-seed 1800s model report, not 1800s DDS wall time",
    "JT20": "Installed CLI/launch/profile here; ordinary clean install in unified verification",
}


def run_scenario(name, output, criteria, domain):
    directory = output / name
    directory.mkdir(parents=True, exist_ok=True)
    script = directory / "input.json"
    script.write_text(json.dumps(script_for(name), indent=2) + "\n")
    share = Path(get_package_share_directory("gca_teleop"))
    config_path = Path(get_package_share_directory("system_bringup")) / "config/simulation.yaml"
    runtime = yaml.safe_load(config_path.read_text())["runtime"]
    budgets = declared_budgets(criteria)
    report = dict(
        scenario=name,
        passed=False,
        clock="host_monotonic_wall",
        source="installed input mapper/session over real DDS, simulated accepted I/O and independent plant",
        physical_input_mapping="NOT_RUN",
        physical_hil="NOT_RUN",
        commands=[],
        cleanup=[],
        script_sha256=hashlib.sha256(script.read_bytes()).hexdigest(),
        installed_inputs=installed_manifest(),
        declared_timeouts=budgets,
        markers={},
    )
    markers = report["markers"]
    observer, launch, arm_thread = None, None, None
    start_ns = time.monotonic_ns()
    pending_faults = set()
    try:
        observer = Observer(domain)
        with tempfile.TemporaryDirectory(prefix="gca-teleop-ipc-") as ipc_dir:
            socket_path = Path(ipc_dir) / "control.sock"

            def status():
                if observer.errors or not observer.thread.is_alive():
                    raise RuntimeError("OBSERVER_FAILED:" + repr(observer.errors))
                result = request(socket_path, timeout_s=0.5)
                if not result.get("accepted"):
                    raise RuntimeError(result.get("error"))
                return result

            def wait(predicate, seconds, label):
                return wait_for(predicate, seconds, label, launch)

            def confirmed():
                result = status()
                return result.get("stop_confirmed") and result["stop_progress"] == "CONFIRMED"

            def operation(op, args=None, allow_failure=False):
                began = time.monotonic_ns()
                entry = dict(
                    entry="installed gca_teleop.cli.request",
                    operation=op,
                    args=args or {},
                    started_ns=began,
                )
                report["commands"].append(entry)
                try:
                    result = request(socket_path, op, args, timeout_s=budgets["management_s"])
                    entry.update(
                        accepted=result.get("accepted"), error=result.get("error"), result=result
                    )
                    if not result.get("accepted") and not allow_failure:
                        raise RuntimeError(op + ":" + result.get("error", "rejected"))
                    return result
                except BaseException as exc:
                    entry["exception"] = type(exc).__name__ + ": " + str(exc)
                    raise
                finally:
                    entry["completed_ns"] = time.monotonic_ns()

            def cli(op, arguments=()):
                command = [
                    "ros2",
                    "run",
                    "gca_teleop",
                    "gca_teleop_cli",
                    "--socket",
                    str(socket_path),
                    op,
                    *arguments,
                ]
                began = time.monotonic_ns()
                completed = subprocess.run(
                    command, cwd=ipc_dir, text=True, capture_output=True, timeout=budgets["cli_s"]
                )
                report["commands"].append(
                    dict(
                        entry="process",
                        command=command,
                        started_ns=began,
                        completed_ns=time.monotonic_ns(),
                        exit_code=completed.returncode,
                        stdout=completed.stdout[-12000:],
                        stderr=completed.stderr[-4000:],
                    )
                )
                if completed.returncode:
                    raise RuntimeError("CLI failed: " + op + ":" + completed.stderr[-500:])
                return json.loads(completed.stdout.strip().splitlines()[-1])

            def fault(name, enabled):
                result = operation("simulation-fault", dict(name=name, enabled=enabled))
                if enabled:
                    pending_faults.add(name)
                else:
                    pending_faults.discard(name)
                return result

            def available_io():
                return [r for r in observer.io_snapshot() if r.get("available")]

            command = [
                "ros2",
                "launch",
                "system_bringup",
                "teleop_simulation.launch.py",
                f"profile_file:={share / 'config/synthetic.yaml'}",
                f"config_file:={config_path}",
                f"script_file:={script}",
                f"socket_path:={socket_path}",
                f"record_directory:={directory / 'input_records'}",
            ]
            report["launch_command"] = command
            with (directory / "launch.log").open("w") as log:
                try:
                    environment = dict(
                        os.environ,
                        ROS_DOMAIN_ID=str(domain),
                        ROS_LOG_DIR=str(directory / "ros_logs"),
                        ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
                    )
                    launch = subprocess.Popen(
                        command,
                        cwd=ipc_dir,
                        env=environment,
                        stdout=log,
                        stderr=subprocess.STDOUT,
                        start_new_session=True,
                    )
                    wait(status, budgets["startup_s"], "teleop owner startup")
                    idle = status()
                    assert (
                        not idle["armed"] and not idle["session_owned"] and idle["sent_count"] == 0
                    ), "ordinary launch moved or claimed"
                    report["idle_launch_status"] = idle
                    active_profile = replace(
                        load_profile(share / "config/synthetic.yaml"),
                        script_file=str(script.resolve()),
                    )
                    assert idle["profile_sha256"] == active_profile.fingerprint, (
                        "installed profile/active override fingerprint mismatch"
                    )
                    report["active_profile_sha256"] = active_profile.fingerprint
                    for op in ("configure", "activate", "claim"):
                        cli(op)
                    wait(
                        lambda: (
                            status()["device_state"] == "DISARMED"
                            and status()["backend"] == "simulation"
                        ),
                        budgets["readiness_s"],
                        "disarmed simulation",
                    )
                    cli("reference", ("--linear-mm", "0", "--rotation-deg", "0"))
                    wait(
                        lambda: status()["rotation_reference_valid"],
                        budgets["reference_s"],
                        "reference",
                    )
                    if name == "slow_arm":
                        fault("arm_pause", True)
                        armed_result, arm_errors = [], []
                        markers["arm_started_ns"] = time.monotonic_ns()

                        def delayed_arm():
                            try:
                                armed_result.append(operation("arm", allow_failure=True))
                            except BaseException as exc:
                                arm_errors.append(str(exc))

                        arm_thread = threading.Thread(target=delayed_arm, daemon=True)
                        arm_thread.start()
                        wait(
                            lambda: any(
                                p["phase"] == "WAITING"
                                and p["received_monotonic_ns"] >= markers["arm_started_ns"]
                                for p in observer.snapshot()[2]
                            ),
                            budgets["arm_barrier_s"],
                            "observable real ARM barrier",
                        )
                        start_ns = operation("script-start")["script_started_ns"]
                        arm_thread.join(budgets["management_s"])
                        assert (
                            not arm_thread.is_alive()
                            and not arm_errors
                            and armed_result
                            and not armed_result[0]["accepted"]
                        ), "slow ARM did not explicitly fail after B"
                        markers["arm_rejected"] = True
                        wait(confirmed, budgets["stop_s"], "B stops pending ARM with new receipt")
                        fault("arm_pause", False)
                        markers["arm_pause_cleared_ns"] = time.monotonic_ns()
                        wait(
                            lambda: (
                                time.monotonic_ns()
                                >= max(
                                    start_ns + 1_800_000_000,
                                    markers["arm_pause_cleared_ns"]
                                    + round(budgets["post_stop_s"] * 1e9),
                                )
                            ),
                            budgets["stop_s"] + 1.8,
                            "post-B late ARM non-revival observation",
                        )
                        assert not status()["armed"] and status()["sent_count"] == 0
                    else:
                        cli("arm")
                        wait(lambda: status()["armed"], budgets["reference_s"], "ARM observed")
                        start_ns = cli("script-start")["script_started_ns"]
                        if name == "control_freeze":
                            wait(
                                lambda: status()["position_mm"] > 0.5,
                                budgets["reference_s"],
                                "movement before control freeze",
                            )
                            fault("control_freeze", True)
                            markers["freeze_started_ns"] = time.monotonic_ns()
                            baseline = wait(
                                lambda: next(
                                    (
                                        r
                                        for r in reversed(available_io())
                                        if r.get("control_paused")
                                        and r["sampled_ns"] >= markers["freeze_started_ns"]
                                        and r["input_valid_until_ns"] > 0
                                    ),
                                    None,
                                ),
                                budgets["stop_s"],
                                "independent frozen-control evidence",
                            )
                            markers["freeze_original_deadline_ns"] = baseline[
                                "input_valid_until_ns"
                            ]
                            boundary = baseline["input_valid_until_ns"] + round(
                                (runtime["io_period_s"] + runtime["io_timeout_s"]) * 1e9
                            )
                            wait(
                                lambda: (
                                    len(
                                        [
                                            r
                                            for r in available_io()
                                            if r.get("control_paused")
                                            and r["sampled_ns"] >= boundary
                                            and abs(r["accepted_velocity_mm_s"]) < 1e-9
                                            and r["pulse_idle"]
                                        ]
                                    )
                                    >= 2
                                ),
                                budgets["stop_s"],
                                "I/O zero after original input deadline while core still frozen",
                            )
                            markers["freeze_cleared_ns"] = time.monotonic_ns()
                            fault("control_freeze", False)
                        if name == "lt_release_restore":
                            wait(
                                lambda: time.monotonic_ns() - start_ns >= 2_300_000_000,
                                3.0,
                                "LT-release locked interval",
                            )
                            wait(confirmed, budgets["stop_s"], "LT release stop evidence")
                            before_restore = status()
                            markers["rotation_reference_lost_on_abort"] = not before_restore[
                                "rotation_reference_valid"
                            ]
                            owner = before_restore["expected"]["session_id"]
                            old_reference = before_restore["expected"]["reference_id"]
                            operation("script-stop")
                            operation("disarm")
                            wait(
                                lambda: confirmed() and not status()["armed"],
                                budgets["stop_s"],
                                "disarm before explicit reference recovery",
                            )
                            if status()["device_state"] == "FAULT":
                                operation("reset")
                            operation("reference", dict(linear_mm=0.0, rotation_deg=0.0))
                            wait(
                                lambda: status()["rotation_reference_valid"],
                                budgets["reference_s"],
                                "reference explicitly restored",
                            )
                            markers["reference_changed"] = (
                                status()["expected"]["reference_id"] != old_reference
                            )
                            recovery_script = directory / "recovery_input.json"
                            recovery_script.write_text(
                                json.dumps(script_for("linear"), indent=2) + "\n"
                            )
                            recovery_profile = directory / "recovery_profile.yaml"
                            profile_data = yaml.safe_load(
                                (share / "config/synthetic.yaml").read_text()
                            )
                            profile_data["script_file"] = str(recovery_script)
                            recovery_profile.write_text(yaml.safe_dump(profile_data))
                            operation("input-config", dict(path=str(recovery_profile)))
                            operation("arm")
                            markers["same_owner_restored"] = (
                                status()["expected"]["session_id"] == owner
                            )
                            markers["recovery_started_ns"] = operation("script-start")[
                                "script_started_ns"
                            ]
                            wait(
                                lambda: (
                                    time.monotonic_ns()
                                    >= markers["recovery_started_ns"]
                                    + round(budgets["recovery_motion_s"] * 1e9)
                                ),
                                budgets["recovery_motion_s"] + budgets["stop_s"],
                                "fresh neutral/LT cycle recovery observation",
                            )
                        else:
                            duration = script_for(name)["duration_s"]
                            wait(
                                lambda: (
                                    time.monotonic_ns() - start_ns
                                    >= round((duration + budgets["script_margin_s"]) * 1e9)
                                ),
                                duration + budgets["stop_s"],
                                "script observation window",
                            )
                            wait(
                                confirmed, budgets["stop_s"], "script STOP new-receipt confirmation"
                            )
                        assert status()["sent_count"] > 0, "input never reached formal Jog path"
                    report["final_status"] = status()
                    report["primary_passed"] = True
                except BaseException as exc:
                    report.update(
                        error=f"{type(exc).__name__}: {exc}",
                        traceback=traceback.format_exc(),
                        primary_passed=False,
                    )
                finally:
                    if launch is not None and launch.poll() is None:
                        for name_to_clear in list(pending_faults):
                            try:
                                fault(name_to_clear, False)
                                report["cleanup"].append(
                                    dict(operation="clear-" + name_to_clear, passed=True)
                                )
                            except BaseException as exc:
                                report["cleanup"].append(
                                    dict(
                                        operation="clear-" + name_to_clear,
                                        passed=False,
                                        error=str(exc),
                                    )
                                )
                        try:
                            operation("stop")
                            wait(
                                confirmed,
                                budgets["stop_s"],
                                "cleanup STOP new-receipt confirmation",
                            )
                            operation("disarm")
                            wait(
                                lambda: confirmed() and not status()["armed"],
                                budgets["stop_s"],
                                "cleanup DISARM confirmation",
                            )
                            operation("release")
                            report["cleanup"].append(
                                dict(operation="stop-disarm-release", passed=True)
                            )
                        except BaseException as exc:
                            report["cleanup"].append(
                                dict(operation="stop-disarm-release", passed=False, error=str(exc))
                            )
                    if arm_thread is not None:
                        arm_thread.join(budgets["management_s"])
                        if arm_thread.is_alive():
                            report["cleanup"].append(
                                dict(operation="join-arm-worker", passed=False)
                            )
                    if launch is not None:
                        report["launch_exit_code"] = stop_owned(launch)
                    observer.close()
            report["cleanup_completed_ns"] = time.monotonic_ns()
    except BaseException as exc:
        report.update(error=f"{type(exc).__name__}: {exc}", traceback=traceback.format_exc())
        if launch is not None and launch.poll() is None:
            report["launch_exit_code"] = stop_owned(launch)
        if observer is not None and observer.thread.is_alive():
            observer.close()
    report["script_started_ns"] = start_ns
    try:
        assert observer is not None, "observer never started"
        rows, teleop, phases = export(directory, observer, start_ns)
        records, core = load_jsonl_records(directory), load_jsonl_records(directory, core=True)
        io = observer.io_snapshot()
        report["checks"] = scenario_checks(
            name, rows, teleop, io, records, phases, start_ns, criteria, runtime, markers
        )
        timeline = stop_timelines(records, core, io, rows, start_ns)
        report["stop_latency"] = timeline["summaries"]
        (directory / "stop_timeline.json").write_text(json.dumps(timeline, indent=2) + "\n")
        timeline_fields = sorted({key for row in timeline["samples"] for key in row})
        if timeline_fields:
            with (directory / "stop_timeline.csv").open("w") as stream:
                writer = csv.DictWriter(stream, fieldnames=timeline_fields)
                writer.writeheader()
                writer.writerows(timeline["samples"])
        report["checks"].append(
            dict(
                name="stop_has_authenticated_serial_and_confirmation_timeline",
                passed=any(
                    r.get("stop_serial")
                    and r.get("confirmation_first_dds_observed_ns")
                    and r.get("independent_io_first_observed_ns")
                    for r in timeline["samples"]
                ),
            )
        )
        report["checks"].append(
            dict(name="observer_healthy", passed=not observer.errors, details=observer.errors)
        )
        startup = [r for r in records if r.get("kind") == "startup"]
        report["checks"].append(
            dict(
                name="active_profile_record_matches_installed_profile",
                passed=bool(startup)
                and startup[0]["profile_sha256"] == report.get("active_profile_sha256"),
            )
        )
        report["dds_samples"], report["teleop_samples"], report["independent_io_samples"] = (
            len(rows),
            len(teleop),
            len(io),
        )
        report["record_files"] = sorted({r["_file"] for r in records + core})
        report["installed_inputs_unchanged"] = report["installed_inputs"] == installed_manifest()
        report["passed"] = (
            bool(report.get("primary_passed"))
            and all(r["passed"] for r in report["checks"])
            and all(r["passed"] for r in report["cleanup"])
            and report.get("launch_exit_code") == 0
            and report["installed_inputs_unchanged"]
        )
    except BaseException as exc:
        report.update(
            passed=False,
            assertion_error=f"{type(exc).__name__}: {exc}",
            analysis_traceback=traceback.format_exc(),
        )
    (directory / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("artifacts/teleop/latest"))
    parser.add_argument("--scenarios", nargs="+", choices=SCENARIOS, default=list(SCENARIOS))
    parser.add_argument(
        "--skip-model",
        action="store_true",
        help="Development only; full acceptance requires model report",
    )
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    criteria_path = (
        Path(get_package_share_directory("system_bringup")) / "config/teleop_acceptance.yaml"
    )
    criteria = yaml.safe_load(criteria_path.read_text())
    report = {
        "passed": False,
        "criteria": criteria,
        "criterion_sha256": hashlib.sha256(criteria_path.read_bytes()).hexdigest(),
        "scenarios": [],
        "coverage_declaration": COVERAGE,
        "installed_inputs": installed_manifest(),
        "physical_input_mapping": "NOT_RUN",
        "wireless_qualification": "NOT_RUN",
        "physical_hil": "NOT_RUN",
    }
    report["sources"] = source_manifest()
    domain = 120 + os.getpid() % 60
    for scenario in args.scenarios:
        print(f"[teleop:{scenario}] running", flush=True)
        result = run_scenario(scenario, output, criteria, domain)
        report["scenarios"].append(result)
        (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        print(f"[teleop:{scenario}] {'PASS' if result['passed'] else 'FAIL'}", flush=True)
    aggregate = {}
    for scenario_result in report["scenarios"]:
        timeline_path = output / scenario_result["scenario"] / "stop_timeline.json"
        if timeline_path.exists():
            for row in json.loads(timeline_path.read_text())["samples"]:
                for key, value in row.items():
                    if key.endswith("_from_request_ms"):
                        aggregate.setdefault(key, []).append(value)
    report["stop_latency"] = {key: latency_stats(values) for key, values in aggregate.items()}
    if not args.skip_model:
        from gca.simulation.teleop_report import run

        try:
            report["model"] = run(output / "model", criteria=criteria)
        except BaseException as exc:
            report["model"] = dict(passed=False, error=str(exc), traceback=traceback.format_exc())
    report["coverage_scope"] = (
        "This runner plus model; source unit/ordinary-install/physical obligations remain separately evidenced in unified verification"
    )
    report["full_acceptance"] = set(args.scenarios) == set(SCENARIOS) and not args.skip_model
    report["passed"] = all(r["passed"] for r in report["scenarios"]) and (
        args.skip_model or report["model"]["passed"]
    )
    report["installed_inputs_unchanged"] = report["installed_inputs"] == installed_manifest()
    report["sources_unchanged"] = report["sources"] == source_manifest()
    report["passed"] = (
        report["passed"] and report["sources_unchanged"] and report["installed_inputs_unchanged"]
    )
    suite = ET.Element(
        "testsuite",
        name="installed_teleop_dds",
        tests=str(len(report["scenarios"]) + 2 + int(not args.skip_model)),
    )
    for result in report["scenarios"]:
        case = ET.SubElement(suite, "testcase", name=result["scenario"])
        if not result["passed"]:
            ET.SubElement(
                case,
                "failure",
                message=result.get("error", result.get("assertion_error", "failed")),
            )
    for label, passed in (
        ("source_inputs_unchanged", report["sources_unchanged"]),
        ("installed_inputs_unchanged", report["installed_inputs_unchanged"]),
    ):
        case = ET.SubElement(suite, "testcase", name=label)
        if not passed:
            ET.SubElement(case, "failure", message=label)
    if not args.skip_model:
        case = ET.SubElement(suite, "testcase", name="independent_model_acceptance")
        if not report["model"]["passed"]:
            ET.SubElement(
                case, "failure", message=report["model"].get("error", "model assertions failed")
            )
    ET.ElementTree(suite).write(output / "junit.xml", encoding="unicode", xml_declaration=True)
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
