"""Deterministic production-runtime Jog -> simulated ports -> independent plant.

This is a clocked I/O harness, not a second motion controller. All trajectory,
PI, phase, limits, state transitions and stopping logic execute in the actual
ControlRuntime. The harness schedules simulated device transactions and feeds
their samples/receipts back exactly as its asynchronous I/O owners do. Formal
ROS/DDS acceptance remains a separate installed-process suite.
"""

from __future__ import annotations

import csv
import hashlib
import json
import xml.etree.ElementTree as ET
from dataclasses import asdict, replace
from math import isfinite, pi, sin, ulp
from pathlib import Path

from gca.config import RuntimeConfig
from gca.domain.manual import ManualInputQuality, host_clock_id
from gca.domain.models import CommandContext, DeviceError, JogCommand, Lease
from gca.runtime import ControlRuntime
from gca.simulation.devices import simulated_devices

# Versioned predeclared model gates; no dependency on a ROS bringup package.
DEFAULT_CRITERIA = {
    "criterion_version": "teleop-controllability-v3",
    "seed": 7,
    "linear": {
        "targets_mm_s": [-2.0, 2.0],
        "slip_fractions": [0.0, 0.1, 0.25],
        "settle_s": 2.0,
        "window_s": 0.5,
        "relative_error": 0.05,
        "quantization_counts": 2,
        "neutral_observation_s": 1.0,
        "neutral_drift_counts": 2,
        "noise_mm": 0.005,
        "encoder_delay_s": 0.02,
        "noisy_window_s": 1.0,
        "noisy_extra_error_mm_s": 0.02,
        "deadband_mm_s": [0.01, 0.5],
        "deadband_targets_mm_s": [-0.25, 0.25],
        "saturation_limit_mm_s": 1.0,
        "saturation_release_s": 1.0,
        "saturation_recovery_s": 2.0,
        "saturation_integral_recovery_mm_s": 0.1,
    },
    "rotation": {
        "fine_deg_s": 0.1,
        "fine_duration_s": 10.0,
        "cumulative_error_pulses": 1,
        "normal_targets_deg_s": [-30.0, 30.0],
        "relative_rate_error": 0.05,
        "quantization_pulses": 2,
        "missed_step_fraction": 0.25,
    },
    "long_model_duration_s": 1800.0,
    "long_model": {
        "common_period_s": 30.0,
        "first_steady_cycle": 1,
        "linear_drift_counts_per_cycle": 2,
        "rotation_drift_pulses_per_cycle": 1,
        "count_truth_error_pulses": 1,
        "final_settle_s": 0.5,
        "repeat_runs": 2,
    },
    "ros": {
        "startup_timeout_s": 20.0,
        "management_timeout_s": 6.0,
        "stop_observation_timeout_s": 2.0,
        "post_stop_observation_s": 0.5,
        "input_max_age_s": 0.15,
        "reader_timeout_s": 0.1,
        "dispatch_hz": 25.0,
    },
}


class ModelClock:
    """Integer model ns: never a wall-clock sleep or ROS simulation clock."""

    def __init__(self):
        self.ns = 1_000_000_000

    def __call__(self):
        return self.ns


def scripted_quality(now_ns, sequence, *, epoch="model-input", age_s=0.15):
    """A genuinely generated model sample with a fixed source deadline."""
    return ManualInputQuality(
        "SCRIPT_SAMPLE",
        "deterministic-runtime-rig",
        epoch,
        sequence,
        now_ns,
        "MODEL_MONOTONIC",
        sequence,
        now_ns,
        now_ns + round(age_s * 1e9),
        host_clock_id(),
        "SYNTHETIC_SAMPLE",
    )


class RuntimeModelRig:
    """Step production control at 10 ms and independent simulated I/O at 5 ms.

    No worker thread or real device is started. Tests can pause either the
    production tick or this I/O scheduler deterministically. Plant state is
    driven exclusively by accepted velocity and simulated emitted edge times.
    """

    def __init__(self, config=None):
        self.config = config or RuntimeConfig()
        self.clock = ModelClock()
        self.runtime = ControlRuntime(
            self.config, devices=simulated_devices(self.config, self.clock), clock=self.clock
        )
        r = self.runtime
        r.servo, r.encoder, r.pulse, r.plant = r._devices
        r.servo.open()
        r.servo.configure()
        r.servo.enable(False)
        r.encoder.open()
        r._pulse_epoch = r.pulse.open().device_epoch
        r._state = replace(r._state, state="DISARMED", configured=True, lifecycle_active=True)
        r._active_lifecycle = True
        r._started = True  # Only to exercise the real non-queue STOP latch.
        self._last_velocity_seq = -1
        self._last_segment_id = self._active_segment_id = None
        self._zero_generation = None
        self.sequence = 0
        self.lease_sequence = 1
        self.step_index = 0
        self.control_paused = False
        self.rows = []
        self.profile_rows = []
        self._origin_ns = self.clock()
        self._io()
        r._tick()
        self.advance(0.04)
        result = r._handle("reference", (3, 0.0, 0.0), self.clock())
        if not result.accepted:
            raise AssertionError(result)
        if not r._handle(
            "arm", Lease("model-session", 1, 1, 5, "model-token"), self.clock()
        ).accepted:
            raise AssertionError("model ARM rejected")
        self._origin_ns = self.clock()

    @property
    def time_s(self):
        return (self.clock() - self._origin_ns) / 1e9

    def jog(self, linear, rotation, *, quality=None, ttl_s=0.14, sequence=None):
        self.sequence = max(self.sequence + 1, sequence or 0)
        context = CommandContext(
            "model-session", "manual-model", f"model-{self.sequence}", 1, self.sequence
        )
        command = JogCommand(
            context,
            linear,
            rotation,
            3,
            "model-token",
            ttl_s,
            True,
            self.clock() + round(ttl_s * 1e9),
            quality or scripted_quality(self.clock(), self.sequence),
        )
        return self.runtime._handle_jog(command, self.clock())

    def _io(self):
        r, now = self.runtime, self.clock()
        deadline = now + round(self.config.io_timeout_s * 1e9)
        r.plant.advance(now)
        if self.step_index % max(1, round(self.config.encoder_period_s / 0.005)) == 0:
            try:
                r._sample.put(r.encoder.read(deadline))
            except DeviceError as exc:
                if exc.code != "ENCODER_WAITING":
                    raise
        enabled = bool(r._state.armed and now < r._lease_deadline)
        if enabled != r._servo_enabled:
            if not enabled or r._armed_stop_serial == r.stop_serial:
                r.servo.enable(enabled)
                r._servo_enabled = enabled
        demand = r._velocity_demand.get()
        if demand and r._valid_output(
            demand.expires_ns, demand.generation, demand.feedback_ns, demand.stop_serial
        ):
            if demand.seq != self._last_velocity_seq:
                accepted = r.servo.write_velocity(demand.velocity, deadline)
                r._io_event("applied", (demand.seq, demand.generation, accepted))
                self._last_velocity_seq = demand.seq
        else:
            r.servo.stop(deadline)
            if self._zero_generation != r._output_generation:
                r._io_event("zero_applied", r._output_generation)
                self._zero_generation = r._output_generation
        r._servo_sample.put(r.servo.read_state(deadline))
        for completion in r.pulse.poll():
            r._io_event("pulse_complete", completion)
            if self._active_segment_id == completion.segment_id:
                self._active_segment_id = None
        if r._pulse_stop.is_set():
            queued = r._pulse_demand.get()
            result = r.pulse.abort()
            r._pulse_stop.clear()
            r._pulse_demand.put(None)
            r._io_event("pulse_aborted", (self._active_segment_id, result))
            self._active_segment_id = None
            if queued and queued[0].segment_id != self._last_segment_id:
                r._io_event("pulse_rejected", queued[0])
        item = r._pulse_demand.get()
        if item and r.pulse.is_idle() and item[0].segment_id != self._last_segment_id:
            segment, expiry = item
            if r._valid_output(
                expiry, segment.generation, stop_serial=segment.stop_serial
            ) and now + round(segment.duration_s * 1e9) <= min(expiry, r._lease_deadline):
                self._active_segment_id = segment.segment_id
                r.pulse.submit(segment)
                r._io_event("pulse_accepted", segment)
                self.profile_rows.append(
                    {
                        "model_s": self.time_s,
                        "segment_id": segment.segment_id,
                        "signed_pulses": segment.signed_pulses,
                        "duration_s": segment.duration_s,
                        "expiry_ns": segment.expires_ns,
                        "generation": segment.generation,
                        "stop_serial": segment.stop_serial,
                        "edge_offsets_us": segment.edge_offsets_us,
                        "start_rate_deg_s": r._diagnostics.get("rotation_profile_start_deg_s", 0),
                        "end_rate_deg_s": r._diagnostics.get("rotation_profile_end_deg_s", 0),
                    }
                )
            else:
                r._io_event("pulse_rejected", segment)
            self._last_segment_id = segment.segment_id
        r._pulse_idle = r.pulse.is_idle()

    def step(self):
        self.clock.ns += 5_000_000
        self.step_index += 1
        self._io()
        if not self.control_paused and self.step_index % 2 == 0:
            self.runtime._tick()

    def advance(self, seconds):
        for _ in range(round(seconds / 0.005)):
            self.step()

    def row(self, requested_linear, requested_rotation):
        state, plant = self.runtime.snapshot(), self.runtime.plant.snapshot()
        return {
            "clock": "model_time",
            "model_s": self.time_s,
            "input_linear_mm_s": requested_linear,
            "input_rotation_deg_s": requested_rotation,
            "limited_linear_mm_s": state.jog_linear_velocity_mm_s,
            "limited_rotation_deg_s": state.jog_rotation_velocity_deg_s,
            "accepted_velocity_mm_s": state.applied_velocity_mm_s,
            "encoder_position_mm": state.position_mm,
            "encoder_velocity_mm_s": state.velocity_mm_s,
            "encoder_seq": state.sample_seq,
            "encoder_received_ns": state.sample_received_ns,
            "output_seq": state.output_seq,
            "output_generation": state.output_generation,
            "stop_serial": state.stop_serial,
            "control_state_seq": state.state_seq,
            "plant_position_mm": plant.position_mm,
            "plant_velocity_mm_s": plant.velocity_mm_s,
            "plant_deadband_mm_s": self.runtime.plant.deadband,
            "plant_velocity_limit_mm_s": self.runtime.plant.max_velocity,
            "confirmed_pulses": state.confirmed_pulses,
            "reserved_pulses": state.reserved_pulses,
            "engine_idle": state.engine_idle,
            "estimated_angle_deg": state.rotation_deg,
            "independent_mechanical_angle_deg": plant.mechanical_angle_deg,
            "planned_phase_deg": state.rotation_planned_phase_deg,
            "profile_rate_deg_s": state.rotation_rate_deg_s,
            "phase_fraction_pulses": self.runtime._jog_rotation.fraction,
            "input_deadline_ns": state.input_valid_until_ns,
            "state": state.state,
            "stop": state.stop_progress,
            "error_code": state.error_code,
            "velocity_integral_mm_s": state.jog_velocity_integral_mm_s,
            "dropped_records": state.dropped_records,
        }

    def run(self, duration_s, intent, *, record_period_s=0.02):
        input_stride = 8  # 25 Hz source frames; each is a fresh model observation.
        sample_stride = max(1, round(record_period_s / 0.005))
        rows = []
        steps = round(duration_s / 0.005)
        requested = (0.0, 0.0)
        for index in range(steps + 1):
            if index % input_stride == 0:
                requested = intent(self.time_s)
                result = self.jog(*requested)
                if not result.accepted:
                    raise AssertionError(f"Jog rejected at {self.time_s}: {result}")
            if index % 100 == 0:
                self.lease_sequence += 1
                result = self.runtime._handle(
                    "renew",
                    Lease("model-session", 1, self.lease_sequence, 5, "model-token"),
                    self.clock(),
                )
                if not result.accepted:
                    raise AssertionError(result)
            if index % sample_stride == 0:
                rows.append(self.row(*requested))
            if self.runtime.snapshot().state == "FAULT":
                raise AssertionError(self.runtime.snapshot())
            if index < steps:
                self.step()
        return rows

    def close(self):
        self.runtime.servo.stop(self.clock() + 1_000_000)
        self.runtime.servo.enable(False)
        self.runtime.pulse.abort()
        self.runtime.servo.close()
        self.runtime.encoder.close()
        self.runtime.pulse.close()
        self.runtime._started = False


def _profile_metrics(rig):
    profiles = rig.profile_rows
    gaps = [
        max(0.0, b["model_s"] - a["model_s"] - a["duration_s"])
        for a, b in zip(profiles, profiles[1:])
    ]
    duration = (
        profiles[-1]["model_s"] + profiles[-1]["duration_s"] - profiles[0]["model_s"]
        if profiles
        else 0
    )
    active = sum(p["duration_s"] for p in profiles)
    return {
        "accepted_segments": len(profiles),
        "max_segment_gap_s": max(gaps, default=0),
        "waveform_occupied_fraction": active / duration if duration else 0,
        "max_planned_rate_deg_s": max(
            (max(abs(p["start_rate_deg_s"]), abs(p["end_rate_deg_s"])) for p in profiles), default=0
        ),
    }


def _export(directory, name, rows, metrics, profiles=()):
    with (directory / f"{name}_accepted_profiles.jsonl").open("w", encoding="utf-8") as stream:
        for profile in profiles:
            stream.write(json.dumps(profile, allow_nan=False) + "\n")
    with (directory / f"{name}.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    with (directory / f"{name}.jsonl").open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, allow_nan=False) + "\n")
    (directory / f"{name}.json").write_text(
        json.dumps(metrics, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    time = [r["model_s"] for r in rows]
    for field in ("input_linear_mm_s", "accepted_velocity_mm_s", "plant_velocity_mm_s"):
        axes[0].plot(time, [r[field] for r in rows], label=field)
    for field in ("estimated_angle_deg", "independent_mechanical_angle_deg", "planned_phase_deg"):
        axes[1].plot(time, [r[field] for r in rows], label=field)
    axes[2].plot(time, [r["phase_fraction_pulses"] for r in rows], label="fractional phase (edges)")
    for axis in axes:
        axis.grid(alpha=0.2)
        axis.legend(fontsize=7)
    axes[0].set_ylabel("Velocity mm/s")
    axes[1].set_ylabel("Angle deg")
    axes[2].set_xlabel("Model seconds; no hardware performance claim")
    fig.suptitle(name)
    fig.tight_layout()
    fig.savefig(directory / f"{name}.png", dpi=130)
    fig.savefig(directory / f"{name}.svg")
    plt.close(fig)


def _source_fingerprints():
    root = Path(__file__).parents[1]
    names = (
        "algorithms/jog_velocity.py",
        "algorithms/jog_rotation.py",
        "algorithms/pulse_timing.py",
        "runtime/control.py",
        "runtime/io_owners.py",
        "simulation/devices.py",
        "simulation/plant.py",
        "simulation/teleop_report.py",
        "config.py",
        "domain/models.py",
        "domain/manual.py",
    )
    return {name: hashlib.sha256((root / name).read_bytes()).hexdigest() for name in names}


def deadband_case(target_mm_s: float, deadband_mm_s: float, criteria: dict | None = None) -> tuple:
    """Short model case; actual runtime PI must cross the plant's unknown deadband.

    The 2 s / 0.5 s / 5%-or-two-count velocity gate and neutral-drift gate are
    unchanged. A 0.25 mm/s intent below the 0.5 mm/s plant deadband forces the
    output/feedback loop to exercise this nonlinearity, rather than bypass it.
    Returns raw rows, metrics and accepted profiles; no plot or worker threads.
    """
    criteria = DEFAULT_CRITERIA if criteria is None else criteria
    if criteria != DEFAULT_CRITERIA:
        raise ValueError("unsupported controllability criteria")
    c = criteria["linear"]
    if (
        type(target_mm_s) not in (int, float)
        or target_mm_s not in c["deadband_targets_mm_s"]
        or type(deadband_mm_s) not in (int, float)
        or deadband_mm_s not in c["deadband_mm_s"]
    ):
        raise ValueError("undeclared deadband model case")
    rig = RuntimeModelRig()
    rig.runtime.plant.deadband = deadband_mm_s
    try:
        rows = rig.run(5, lambda t: (target_mm_s if t < 3 else 0.0, 0.0))
        window = [r for r in rows if c["settle_s"] <= r["model_s"] <= c["settle_s"] + c["window_s"]]
        mean = sum(r["plant_velocity_mm_s"] for r in window) / len(window)
        tolerance = max(
            c["relative_error"] * abs(target_mm_s),
            c["quantization_counts"] * rig.config.encoder_mm_per_count / c["window_s"],
        )
        neutral = [r["plant_position_mm"] for r in rows if r["model_s"] >= 4]
        drift = max(neutral) - min(neutral)
        error = abs(mean - target_mm_s)
        metrics = {
            "scenario": f"deadband_{target_mm_s:+.2f}_{deadband_mm_s:.2f}",
            "clock": "model_time",
            "target_mm_s": target_mm_s,
            "deadband_mm_s": deadband_mm_s,
            "mean_independent_speed_mm_s": mean,
            "absolute_error_mm_s": error,
            "threshold_mm_s": tolerance,
            "steady_window_s": [c["settle_s"], c["settle_s"] + c["window_s"]],
            "neutral_drift_mm": drift,
            "neutral_drift_threshold_mm": c["neutral_drift_counts"]
            * rig.config.encoder_mm_per_count,
            "max_actual_accepted_velocity_mm_s": max(
                abs(r["accepted_velocity_mm_s"]) for r in rows
            ),
            "passed": error <= tolerance
            and drift <= c["neutral_drift_counts"] * rig.config.encoder_mm_per_count,
        }
        return rows, metrics, rig.profile_rows
    finally:
        rig.close()


def saturation_case(target_mm_s: float, criteria: dict | None = None) -> tuple:
    """Apply a 1 mm/s independent actuator cap, then release it at 1 model s.

    Recovery uses the original 2 s / 0.5 s / 5%-or-two-count velocity gate after
    release, plus a predeclared 0.1 mm/s remaining PI integral bound. The cap is
    enforced by the accepted-I/O plant, never assigned to encoder/true velocity.
    """
    criteria = DEFAULT_CRITERIA if criteria is None else criteria
    if criteria != DEFAULT_CRITERIA:
        raise ValueError("unsupported controllability criteria")
    c = criteria["linear"]
    if type(target_mm_s) not in (int, float) or target_mm_s not in c["targets_mm_s"]:
        raise ValueError("undeclared saturation model case")
    release_s = c["saturation_release_s"]
    recovered_s = release_s + c["saturation_recovery_s"]
    rig = RuntimeModelRig()
    released_limit = rig.runtime.plant.max_velocity

    def intent(t):
        rig.runtime.plant.max_velocity = (
            c["saturation_limit_mm_s"] if t < release_s else released_limit
        )
        return (target_mm_s if t < 4 else 0.0, 0.0)

    try:
        rows = rig.run(6, intent)
        capped = [r for r in rows if 0.4 <= r["model_s"] < release_s]
        window = [r for r in rows if recovered_s <= r["model_s"] <= recovered_s + c["window_s"]]
        mean = sum(r["plant_velocity_mm_s"] for r in window) / len(window)
        error = abs(mean - target_mm_s)
        tolerance = max(
            c["relative_error"] * abs(target_mm_s),
            c["quantization_counts"] * rig.config.encoder_mm_per_count / c["window_s"],
        )
        remaining_i = max(abs(r["velocity_integral_mm_s"]) for r in window)
        maximum_i = max(abs(r["velocity_integral_mm_s"]) for r in rows)
        actual_cap = max(abs(r["accepted_velocity_mm_s"]) for r in capped)
        released = any(
            abs(r["accepted_velocity_mm_s"]) > c["saturation_limit_mm_s"] for r in window
        )
        moving = any(
            abs(r["plant_velocity_mm_s"]) > 0.5 * c["saturation_limit_mm_s"] for r in capped
        )
        peak = max(abs(r["plant_velocity_mm_s"]) for r in rows)
        neutral = [r["plant_position_mm"] for r in rows if r["model_s"] >= 5]
        drift = max(neutral) - min(neutral)
        metrics = {
            "scenario": f"saturation_release_{target_mm_s:+.1f}",
            "clock": "model_time",
            "target_mm_s": target_mm_s,
            "actuator_cap_mm_s": c["saturation_limit_mm_s"],
            "release_model_s": release_s,
            "released_actuator_limit_mm_s": released_limit,
            "recovery_window_s": [recovered_s, recovered_s + c["window_s"]],
            "max_accepted_under_cap_mm_s": actual_cap,
            "mean_independent_recovered_speed_mm_s": mean,
            "absolute_error_mm_s": error,
            "threshold_mm_s": tolerance,
            "max_integral_mm_s": maximum_i,
            "integral_limit_mm_s": rig.config.jog_integral_limit_mm_s,
            "remaining_integral_mm_s": remaining_i,
            "integral_recovery_threshold_mm_s": c["saturation_integral_recovery_mm_s"],
            "sampled_peak_independent_speed_mm_s": peak,
            "record_period_s": 0.02,
            "neutral_drift_mm": drift,
            "passed": actual_cap <= c["saturation_limit_mm_s"]
            and moving
            and released
            and error <= tolerance
            and maximum_i <= rig.config.jog_integral_limit_mm_s
            and remaining_i <= c["saturation_integral_recovery_mm_s"]
            and peak <= rig.config.max_linear_mm_s
            and drift <= c["neutral_drift_counts"] * rig.config.encoder_mm_per_count,
        }
        return rows, metrics, rig.profile_rows
    finally:
        rig.close()


def fine_rotation_metrics(rows, velocity, duration_s, edge_deg, criteria):
    """Compare settled edges with an independent trapezoidal limited-rate integral."""
    limited_integral = (
        sum(
            (a["limited_rotation_deg_s"] + b["limited_rotation_deg_s"])
            * 0.5
            * (b["model_s"] - a["model_s"])
            for a, b in zip(rows, rows[1:])
        )
        / edge_deg
    )
    confirmed = rows[-1]["confirmed_pulses"]
    error = abs(confirmed - limited_integral)
    raw_integral = velocity * duration_s / edge_deg
    return {
        "ideal_input_integral_edges": raw_integral,
        "independent_limited_integral_edges": limited_integral,
        "raw_input_integral_error_edges": abs(confirmed - raw_integral),
        "count_error_edges": error,
        "count_error_threshold_edges": criteria["rotation"]["cumulative_error_pulses"],
        "passed": error <= criteria["rotation"]["cumulative_error_pulses"] and confirmed != 0,
    }


# Explicitly model-valued fields. UUIDs, boot identities and host timestamps are
# intentionally excluded; control/output/sample sequences are part of the hash.
_TRACE_FIELDS = (
    "model_s",
    "input_linear_mm_s",
    "input_rotation_deg_s",
    "limited_linear_mm_s",
    "limited_rotation_deg_s",
    "accepted_velocity_mm_s",
    "encoder_position_mm",
    "encoder_velocity_mm_s",
    "encoder_seq",
    "encoder_received_ns",
    "plant_position_mm",
    "plant_velocity_mm_s",
    "plant_deadband_mm_s",
    "plant_velocity_limit_mm_s",
    "confirmed_pulses",
    "reserved_pulses",
    "engine_idle",
    "estimated_angle_deg",
    "independent_mechanical_angle_deg",
    "planned_phase_deg",
    "profile_rate_deg_s",
    "phase_fraction_pulses",
    "input_deadline_ns",
    "velocity_integral_mm_s",
    "output_seq",
    "output_generation",
    "stop_serial",
    "control_state_seq",
    "dropped_records",
    "state",
    "stop",
    "error_code",
)


def numerical_trace_hash(rows) -> str:
    """Hash sampled numerical/state rows, never UUID/host metadata or unrecorded ticks."""
    digest = hashlib.sha256()
    for row in rows:
        payload = {field: row[field] for field in _TRACE_FIELDS}
        digest.update(
            json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
        )
        digest.update(b"\n")
    return digest.hexdigest()


def long_trajectory_metrics(rows, duration_s, config, criteria):
    """Evaluate predeclared periodic drift and counted-edge/plant conservation.

    Velocity Jog is not a position HOLD. Each complete common period receives
    the same count-derived error budget; the cumulative bound is the triangle
    inequality N times that budget. Initial-to-final offset is reported too.
    Active-wave truth belongs between confirmed and signed reserved endpoints;
    only idle/unreserved rows assert completed-count equality.
    """
    c = criteria["long_model"]
    edge_deg = 360 * config.catheter_per_motor_ratio / config.pulses_per_motor_rev
    period_ns = round(c["common_period_s"] * 1e9)
    first_ns = period_ns * c["first_steady_cycle"]
    end_ns = round(duration_s * 1e9)
    endpoints = {}
    for row in rows:
        ns = round(row["model_s"] * 1e9)
        if first_ns <= ns <= end_ns and ns % period_ns == 0:
            endpoints[ns] = row
    points = [endpoints[ns] for ns in sorted(endpoints)]
    intervals = max(0, len(points) - 1)
    linear_per_cycle = c["linear_drift_counts_per_cycle"] * config.encoder_mm_per_count
    angular_per_cycle = c["rotation_drift_pulses_per_cycle"] * edge_deg
    dx = [abs(b["plant_position_mm"] - a["plant_position_mm"]) for a, b in zip(points, points[1:])]
    da = [
        abs(b["independent_mechanical_angle_deg"] - a["independent_mechanical_angle_deg"])
        for a, b in zip(points, points[1:])
    ]
    cumulative_x = max(
        (abs(r["plant_position_mm"] - points[0]["plant_position_mm"]) for r in points), default=0
    )
    cumulative_a = max(
        (
            abs(
                r["independent_mechanical_angle_deg"]
                - points[0]["independent_mechanical_angle_deg"]
            )
            for r in points
        ),
        default=0,
    )
    settled_errors, envelope_errors = [], []
    for row in rows:
        confirmed, reserved = row["estimated_angle_deg"], row["reserved_pulses"] * edge_deg
        truth = row["independent_mechanical_angle_deg"]
        low, high = min(confirmed, confirmed + reserved), max(confirmed, confirmed + reserved)
        envelope_errors.append(max(low - truth, truth - high, 0) / edge_deg)
        if row["engine_idle"] and row["reserved_pulses"] == 0:
            settled_errors.append(abs(confirmed - truth) / edge_deg)
    observe_start = duration_s + c["final_settle_s"]
    neutral = [
        r
        for r in rows
        if observe_start
        <= r["model_s"]
        <= observe_start + criteria["linear"]["neutral_observation_s"]
    ]
    neutral_x = max((r["plant_position_mm"] for r in neutral), default=0) - min(
        (r["plant_position_mm"] for r in neutral), default=0
    )
    neutral_a = max((r["independent_mechanical_angle_deg"] for r in neutral), default=0) - min(
        (r["independent_mechanical_angle_deg"] for r in neutral), default=0
    )
    neutral_bound = criteria["linear"]["neutral_drift_counts"] * config.encoder_mm_per_count
    # Predeclared numerical rounding allowance, not an extra physical count.
    # A fixed 16-ULP-per-step policy covers small ledger/comparison roundoff
    # at the observed magnitude. Report it separately; this is not a physical
    # tolerance or a proof of a worst-case floating-point condition bound.
    steps = round(
        (duration_s + c["final_settle_s"] + criteria["linear"]["neutral_observation_s"]) / 0.005
    )
    linear_roundoff = 16 * (steps + 1) * ulp(max(abs(r["plant_position_mm"]) for r in rows) + 1)
    angle_roundoff = (
        16 * (steps + 1) * ulp(max(abs(r["independent_mechanical_angle_deg"]) for r in rows) + 1)
    )
    neutral_settled = bool(neutral) and all(
        abs(r["plant_velocity_mm_s"]) <= config.velocity_tolerance_mm_s
        and r["engine_idle"]
        and r["reserved_pulses"] == 0
        for r in neutral
    )
    checks = {
        "complete_common_period_coverage": intervals >= 1,
        "per_cycle_linear_drift": max(dx, default=0) <= linear_per_cycle + linear_roundoff,
        "per_cycle_angular_drift": max(da, default=0) <= angular_per_cycle + angle_roundoff,
        "cumulative_linear_drift": cumulative_x <= intervals * linear_per_cycle + linear_roundoff,
        "cumulative_angular_drift": cumulative_a <= intervals * angular_per_cycle + angle_roundoff,
        "completed_count_matches_no_loss_plant": bool(settled_errors)
        and max(settled_errors) <= c["count_truth_error_pulses"] + angle_roundoff / edge_deg,
        "inflight_count_reservation_contains_plant": max(envelope_errors, default=0)
        <= c["count_truth_error_pulses"] + angle_roundoff / edge_deg,
        "neutral_is_settled": neutral_settled,
        "neutral_linear_drift": bool(neutral) and neutral_x <= neutral_bound + linear_roundoff,
        "neutral_angular_drift": bool(neutral)
        and neutral_a <= c["count_truth_error_pulses"] * edge_deg + angle_roundoff,
        "phase_remainder_bounded": all(abs(r["phase_fraction_pulses"]) <= 0.500001 for r in rows),
        "no_fault": all(r["state"] != "FAULT" for r in rows),
    }
    return {
        "clock": "model_time",
        "common_period_s": c["common_period_s"],
        "steady_period_endpoints": [
            {
                "model_s": r["model_s"],
                "plant_position_mm": r["plant_position_mm"],
                "plant_angle_deg": r["independent_mechanical_angle_deg"],
            }
            for r in points
        ],
        "period_interval_count": intervals,
        "float_roundoff_ops_per_model_step": 16,
        "float_roundoff_guard_mm": linear_roundoff,
        "float_roundoff_guard_deg": angle_roundoff,
        "linear_period_drift_threshold_mm": linear_per_cycle,
        "angular_period_drift_threshold_deg": angular_per_cycle,
        "max_period_linear_drift_mm": max(dx, default=0),
        "max_period_angular_drift_deg": max(da, default=0),
        "max_cumulative_linear_drift_mm": cumulative_x,
        "max_cumulative_angular_drift_deg": cumulative_a,
        "cumulative_linear_budget_mm": intervals * linear_per_cycle,
        "cumulative_angular_budget_deg": intervals * angular_per_cycle,
        "full_trace_linear_offset_mm": rows[-1]["plant_position_mm"] - rows[0]["plant_position_mm"],
        "full_trace_angular_offset_deg": rows[-1]["independent_mechanical_angle_deg"]
        - rows[0]["independent_mechanical_angle_deg"],
        "max_completed_count_truth_error_pulses": max(settled_errors, default=0),
        "completed_count_truth_sample_count": len(settled_errors),
        "max_inflight_envelope_error_pulses": max(envelope_errors, default=0),
        "neutral_observation_start_s": observe_start,
        "neutral_observation_s": criteria["linear"]["neutral_observation_s"],
        "neutral_linear_drift_mm": neutral_x,
        "neutral_angular_drift_deg": neutral_a,
        "checks": checks,
        "passed": all(checks.values()),
    }


def _validate_long_duration(duration_s, criteria):
    if type(duration_s) not in (int, float) or not isfinite(duration_s):
        raise ValueError("finite model duration required")
    period = criteria["long_model"]["common_period_s"]
    ns, period_ns = round(duration_s * 1e9), round(period * 1e9)
    if duration_s < 2 * period or duration_s > criteria["long_model_duration_s"] or ns % period_ns:
        raise ValueError(
            "model duration must contain 2+ whole common periods within declared report capacity"
        )


def long_sampling_case(duration_s: float, criteria=None) -> tuple:
    """Run both same-seed trajectories; 60 s development is not 1800 s acceptance.

    Returns per-run raw rows/profiles and aggregated metrics. No wall-clock
    sleep, worker thread, real device or target-to-plant assignment is used.
    """
    criteria = DEFAULT_CRITERIA if criteria is None else criteria
    if criteria != DEFAULT_CRITERIA:
        raise ValueError("unsupported controllability criteria")
    _validate_long_duration(duration_s, criteria)
    runs, analyses, hashes = [], [], []
    for _ in range(criteria["long_model"]["repeat_runs"]):
        rig = RuntimeModelRig(RuntimeConfig(simulation_slip=0.1))
        try:
            rows = rig.run(
                duration_s,
                lambda t: (0.5 * sin(2 * pi * t / 10), 5 * sin(2 * pi * t / 6)),
                record_period_s=0.1,
            )
            rows.extend(
                rig.run(
                    criteria["long_model"]["final_settle_s"],
                    lambda t: (0.0, 0.0),
                    record_period_s=0.1,
                )
            )
            rows.extend(
                rig.run(
                    criteria["linear"]["neutral_observation_s"],
                    lambda t: (0.0, 0.0),
                    record_period_s=0.1,
                )
            )
            analysis = long_trajectory_metrics(rows, duration_s, rig.config, criteria)
            analysis.update(
                requested_stimulus_model_duration_s=duration_s,
                actual_stimulus_model_duration_s=max(
                    r["model_s"] for r in rows if r["model_s"] <= duration_s
                ),
                actual_observed_model_duration_s=rows[-1]["model_s"] - rows[0]["model_s"],
                sample_stride_s=0.1,
                io_step_s=0.005,
                retained_control_records=len(rig.runtime.records()),
                dropped_control_records=rig.runtime.snapshot().dropped_records,
                max_fraction_edges=max(abs(r["phase_fraction_pulses"]) for r in rows),
                retained_rows=len(rows),
            )
            analysis["checks"]["bounded_control_records"] = len(rig.runtime.records()) <= 4000
            analysis["checks"]["complete_actual_duration"] = (
                analysis["actual_stimulus_model_duration_s"] == duration_s
                and analysis["actual_observed_model_duration_s"]
                == duration_s
                + criteria["long_model"]["final_settle_s"]
                + criteria["linear"]["neutral_observation_s"]
            )
            expected_rows = sum(
                round(s / 0.1) + 1
                for s in (
                    duration_s,
                    criteria["long_model"]["final_settle_s"],
                    criteria["linear"]["neutral_observation_s"],
                )
            )
            analysis["checks"]["recorded_sample_count_matches_schedule"] = (
                len(rows) == expected_rows
            )
            analysis["expected_recorded_rows"] = expected_rows
            analysis["passed"] = all(analysis["checks"].values())
            analyses.append(analysis)
            hashes.append(numerical_trace_hash(rows))
            runs.append((rows, rig.profile_rows))
        finally:
            rig.close()
    metrics = {
        "scenario": "long_sampling",
        "clock": "model_time",
        "duration_s": duration_s,
        "full_30_minutes": duration_s >= criteria["long_model_duration_s"],
        "repeat_runs": len(runs),
        "sampled_trace_sha256": hashes,
        "sample_stride_s": 0.1,
        "hash_scope": "recorded 0.1s numeric/state samples and neutral boundaries, not every 5ms I/O tick",
        "raw_exported_runs": [1],
        "hash_fields": list(_TRACE_FIELDS),
        "same_seed_exact_repeat": len(set(hashes)) == 1,
        "run_metrics": analyses,
        "passed": len(set(hashes)) == 1 and all(a["passed"] for a in analyses),
    }
    return runs, metrics


def _scenario_names(criteria):
    linear, rotation = criteria["linear"], criteria["rotation"]
    names = [
        f"linear_{direction:+d}_slip_{slip:.2f}"
        for direction in (1, -1)
        for slip in linear["slip_fractions"]
    ]
    names += [
        f"{name}_{direction:+d}"
        for direction in (-1, 1)
        for name in ("noise", "delay", "noise_delay")
    ]
    names += [
        f"deadband_{target:+.2f}_{db:.2f}"
        for target in linear["deadband_targets_mm_s"]
        for db in linear["deadband_mm_s"]
    ]
    names += [f"saturation_release_{target:+.1f}" for target in linear["targets_mm_s"]]
    names += [
        f"rotation_{v:+.1f}" for v in (rotation["fine_deg_s"], *rotation["normal_targets_deg_s"])
    ]
    return names + ["rotation_missed_steps", "long_sampling"]


def write_model_junit(
    output: Path, scenarios: list[dict], criteria: dict, duration_s: float
) -> None:
    """Emit model-only cases without mislabelling model seconds as wall time."""
    suite = ET.Element(
        "testsuite",
        name="gca_teleop_independent_model",
        tests=str(len(scenarios)),
        failures=str(sum(not r["passed"] for r in scenarios)),
        errors="0",
        skipped="0",
    )
    properties = ET.SubElement(suite, "properties")
    for name, value in {
        "clock": "model_time",
        "criterion_version": criteria["criterion_version"],
        "full_duration": str(
            any(
                r.get("scenario") == "long_sampling"
                and r.get("full_30_minutes") is True
                and r.get("repeat_runs") == criteria["long_model"]["repeat_runs"]
                for r in scenarios
            )
        ).lower(),
        "model_duration_s": str(duration_s),
    }.items():
        ET.SubElement(properties, "property", name=name, value=value)
    for result in scenarios:
        case = ET.SubElement(suite, "testcase", name=result["scenario"], classname="model_time")
        if not result["passed"]:
            failure = ET.SubElement(
                case, "failure", message=result.get("error", "predeclared model criteria failed")
            )
            failure.text = json.dumps(result, sort_keys=True, allow_nan=False)
    ET.ElementTree(suite).write(output / "junit.xml", encoding="utf-8", xml_declaration=True)


def run(output: Path, criteria=None, *, long_duration_s=1800.0) -> dict:
    """Export 23 model cases, two full-duration repeats, and separate JUnit.

    60/90/... s whole-period runs are development only. Duration is bounded by
    the declared 1800 s report capacity; full acceptance uses both 1800 s runs.
    A scenario exception writes explicit failed/not-completed cases and returns
    passed=False so the installed suite exits nonzero; resources close in each
    rig's finally block. No model duration is used as JUnit elapsed wall time.
    """
    criteria = DEFAULT_CRITERIA if criteria is None else criteria
    if criteria != DEFAULT_CRITERIA:
        raise ValueError("unsupported controllability criteria")
    _validate_long_duration(long_duration_s, criteria)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    results = []
    pending = [
        {"scenario": name, "clock": "model_time", "passed": False, "status": "RUN_NOT_COMPLETED"}
        for name in _scenario_names(criteria)
    ]
    write_model_junit(output, pending, criteria, long_duration_s)
    (output / "summary.json").write_text(
        json.dumps(
            {
                "passed": False,
                "status": "RUNNING",
                "criterion_version": criteria["criterion_version"],
                "clock": "model_time",
            }
        )
        + "\n",
        encoding="utf-8",
    )
    try:
        summary = _run_report(output, criteria, long_duration_s=long_duration_s, results=results)
    except Exception as exc:
        completed = {r["scenario"] for r in results}
        for name in _scenario_names(criteria):
            if name not in completed:
                results.append(
                    {
                        "scenario": name,
                        "clock": "model_time",
                        "passed": False,
                        "status": "NOT_COMPLETED_AFTER_EXCEPTION",
                        "error": type(exc).__name__ + ": " + str(exc),
                    }
                )
        summary = {
            "criterion_version": criteria["criterion_version"],
            "criteria": criteria,
            "clock": "model_time",
            "hardware_used": False,
            "source_sha256": _source_fingerprints(),
            "full_acceptance_model_duration": long_duration_s >= criteria["long_model_duration_s"],
            "scope": "actual_core_runtime_and_simulated_ports_not_ROS_DDS",
            "scenarios": results,
            "passed": False,
            "error": type(exc).__name__ + ": " + str(exc),
        }
    summary["requested_model_duration_s"] = long_duration_s
    summary["full_acceptance_model_duration"] = any(
        r.get("scenario") == "long_sampling"
        and r.get("full_30_minutes") is True
        and r.get("repeat_runs") == criteria["long_model"]["repeat_runs"]
        for r in results
    )
    summary["passed"] = (
        summary["passed"]
        and len(results) == len(_scenario_names(criteria))
        and {r["scenario"] for r in results} == set(_scenario_names(criteria))
    )
    write_model_junit(output, results, criteria, long_duration_s)
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    return summary


def _run_report(output: Path, criteria=None, *, long_duration_s=1800.0, results=None) -> dict:
    """Generate predeclared controllability evidence; false passed means failure.

    Default includes the required 30 model minutes. A shorter duration may be
    used by unit tests but is labelled as development, not full acceptance.
    """
    if (
        type(long_duration_s) not in (int, float)
        or not isfinite(long_duration_s)
        or long_duration_s <= 0
    ):
        raise ValueError("positive finite model duration required")
    criteria = DEFAULT_CRITERIA if criteria is None else criteria
    if criteria != DEFAULT_CRITERIA:
        raise ValueError(
            "unsupported or relaxed controllability criteria; revise version and tests first"
        )
    linear, rotation = criteria["linear"], criteria["rotation"]
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    results = [] if results is None else results
    for direction in (1, -1):
        for slip in linear["slip_fractions"]:
            name = f"linear_{direction:+d}_slip_{slip:.2f}"
            rig = RuntimeModelRig(RuntimeConfig(simulation_slip=slip))
            try:
                rows = rig.run(5.0, lambda t, sign=direction: (2.0 * sign if t < 3.0 else 0.0, 0.0))
                steady = [
                    r["plant_velocity_mm_s"]
                    for r in rows
                    if linear["settle_s"] <= r["model_s"] <= linear["settle_s"] + linear["window_s"]
                ]
                mean = sum(steady) / len(steady)
                error = abs(mean - 2 * direction)
                stable = [r["plant_position_mm"] for r in rows if r["model_s"] >= 4.0]
                drift = max(stable) - min(stable)
                threshold = max(
                    linear["relative_error"] * 2,
                    linear["quantization_counts"]
                    * rig.config.encoder_mm_per_count
                    / linear["window_s"],
                )
                metrics = {
                    "scenario": name,
                    "clock": "model_time",
                    "slip": slip,
                    "target_mm_s": 2 * direction,
                    "steady_window_s": [
                        linear["settle_s"],
                        linear["settle_s"] + linear["window_s"],
                    ],
                    "mean_independent_speed_mm_s": mean,
                    "absolute_error_mm_s": error,
                    "threshold_mm_s": threshold,
                    "neutral_drift_mm": drift,
                    "neutral_drift_threshold_mm": linear["neutral_drift_counts"]
                    * rig.config.encoder_mm_per_count,
                    "passed": error <= threshold
                    and drift <= linear["neutral_drift_counts"] * rig.config.encoder_mm_per_count,
                }
                _export(output, name, rows, metrics, rig.profile_rows)
                results.append(metrics)
            finally:
                rig.close()
    for direction in (-1, 1):
        for name, noise, delay in (
            ("noise", linear["noise_mm"], 0.0),
            ("delay", 0.0, linear["encoder_delay_s"]),
            ("noise_delay", linear["noise_mm"], linear["encoder_delay_s"]),
        ):
            name = f"{name}_{direction:+d}"
            rig = RuntimeModelRig(
                RuntimeConfig(simulation_noise_mm=noise, simulation_encoder_delay_s=delay)
            )
            try:
                rows = rig.run(4, lambda t: (direction * 2, 0))
                window = [
                    r
                    for r in rows
                    if linear["settle_s"]
                    <= r["model_s"]
                    <= linear["settle_s"] + linear["noisy_window_s"]
                ]
                mean = sum(r["plant_velocity_mm_s"] for r in window) / len(window)
                threshold = (
                    max(
                        linear["relative_error"] * 2,
                        linear["quantization_counts"]
                        * rig.config.encoder_mm_per_count
                        / linear["noisy_window_s"],
                    )
                    + linear["noisy_extra_error_mm_s"]
                )
                metrics = {
                    "scenario": name,
                    "clock": "model_time",
                    "target_mm_s": direction * 2,
                    "noise_mm": noise,
                    "encoder_delay_s": delay,
                    "mean_independent_speed_mm_s": mean,
                    "absolute_error_mm_s": abs(mean - direction * 2),
                    "threshold_mm_s": threshold,
                    "passed": abs(mean - direction * 2) <= threshold,
                }
                _export(output, name, rows, metrics, rig.profile_rows)
                results.append(metrics)
            finally:
                rig.close()
    for target in linear["deadband_targets_mm_s"]:
        for deadband in linear["deadband_mm_s"]:
            rows, metrics, profiles = deadband_case(target, deadband, criteria)
            _export(output, metrics["scenario"], rows, metrics, profiles)
            results.append(metrics)
    for target in linear["targets_mm_s"]:
        rows, metrics, profiles = saturation_case(target, criteria)
        _export(output, metrics["scenario"], rows, metrics, profiles)
        results.append(metrics)
    for velocity, duration in (
        (rotation["fine_deg_s"], rotation["fine_duration_s"]),
        *((v, 4.0) for v in rotation["normal_targets_deg_s"]),
    ):
        name = f"rotation_{velocity:+.1f}"
        rig = RuntimeModelRig()
        try:
            rows = rig.run(duration, lambda t, value=velocity: (0.0, value))
            # Neutral permits only bounded deceleration; no LT abort is used
            # to falsify the fully-settled counting/low-rate measurement.
            rows.extend(rig.run(0.5, lambda t: (0.0, 0.0)))
            last = rows[-1]
            if abs(velocity) < 1:
                extra = fine_rotation_metrics(
                    rows, velocity, duration, rig.runtime._quantizer.angle(1), criteria
                )
                passed = extra.pop("passed")
            else:
                window = [r for r in rows if 2 <= r["model_s"] <= 3]
                average = (window[-1]["estimated_angle_deg"] - window[0]["estimated_angle_deg"]) / (
                    window[-1]["model_s"] - window[0]["model_s"]
                )
                threshold = max(
                    rotation["relative_rate_error"] * abs(velocity),
                    rotation["quantization_pulses"] * rig.runtime._quantizer.angle(1),
                )
                passed = abs(average - velocity) <= threshold
                extra = {"average_confirmed_deg_s": average, "threshold_deg_s": threshold}
            metrics = {
                "scenario": name,
                "clock": "model_time",
                "requested_deg_s": velocity,
                "confirmed_pulses": last["confirmed_pulses"],
                "passed": passed,
                **_profile_metrics(rig),
                **extra,
            }
            _export(output, name, rows, metrics, rig.profile_rows)
            results.append(metrics)
        finally:
            rig.close()
    # Paired no-loss rotation is above. This predeclared hidden fault is an
    # observability test: the open-loop runtime must not fabricate detection.
    rig = RuntimeModelRig()
    try:
        missed_fraction = rotation["missed_step_fraction"]
        rig.runtime.plant.missed_step_fraction = missed_fraction
        rows = rig.run(1.0, lambda t: (0.0, 30.0))
        rows.extend(rig.run(0.5, lambda t: (0.0, 0.0)))
        final = rows[-1]
        expected_truth = (1 - missed_fraction) * final["estimated_angle_deg"]
        truth_error = abs(final["independent_mechanical_angle_deg"] - expected_truth)
        state = rig.runtime.snapshot()
        metrics = {
            "scenario": "rotation_missed_steps",
            "clock": "model_time",
            "missed_step_fraction": missed_fraction,
            "open_loop_can_detect_missed_steps": False,
            "software_confirmed_angle_deg": final["estimated_angle_deg"],
            "independent_mechanical_angle_deg": final["independent_mechanical_angle_deg"],
            "independent_truth_prediction_error_deg": truth_error,
            "reference_remains_valid_without_feedback": state.rotation_reference_valid,
            "passed": truth_error <= 1e-9
            and state.rotation_reference_valid
            and abs(final["estimated_angle_deg"] - final["independent_mechanical_angle_deg"]) > 1,
        }
        _export(output, "rotation_missed_steps", rows, metrics, rig.profile_rows)
        results.append(metrics)
    finally:
        rig.close()
    runs, metrics = long_sampling_case(long_duration_s, criteria)
    # First raw trace is sufficient to reproduce both sampled hashes. Retain
    # actual per-run duration/count/checks/hash without duplicate large exports.
    _export(output, "long_sampling", runs[0][0], metrics, runs[0][1])
    results.append(metrics)
    summary = {
        "criterion_version": criteria["criterion_version"],
        "criteria": criteria,
        "clock": "model_time",
        "seed": criteria["seed"],
        "hardware_used": False,
        "scope": "actual_core_runtime_and_simulated_ports_not_ROS_DDS",
        "rotation_constraints": {
            "intent": "long-term average deg/s; profile peak separately reported",
            "software": "finite profile phase/rate/acceleration and integer-us edge error",
            "gap": "single-wave handoff gaps recorded; global mechanical acceleration unverified",
            "hardware": "HIL required; open-loop step loss not observable",
        },
        "full_acceptance_model_duration": long_duration_s >= 1800,
        "configuration": asdict(RuntimeConfig()),
        "source_sha256": _source_fingerprints(),
        "scenarios": results,
        "passed": all(r["passed"] for r in results),
    }
    return summary
