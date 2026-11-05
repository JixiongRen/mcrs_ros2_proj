#!/usr/bin/env python3
"""Export deterministic algorithm/plant evidence without ROS or hardware.

The formal installed ROS execution path is tested separately by
simulation_suite.py. This harness reuses the installed production trajectory,
controller, quantizer, estimator and simulated device ports with an injected
model clock. Its results describe model assumptions, never measured hardware.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import traceback
from pathlib import Path

from gca.algorithms.control import PositionController, boundary_velocity, clamp, slew
from gca.algorithms.trajectory import PulseQuantizer, Trapezoid
from gca.domain.models import PulseSegment
from gca.estimation.linear import LinearEstimator
from gca.simulation.devices import SimEncoder, SimPulseEngine, SimServo
from gca.simulation.plant import MechanicalPlant


class ModelClock:
    """Exact integer-step model clock, entirely independent of wall time."""

    def __init__(self):
        self.now_ns = 0

    def __call__(self):
        return self.now_ns

    def at_step(self, step: int, period_ns: int):
        self.now_ns = step * period_ns


def _save_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        raise ValueError("a model scenario produced no evidence rows")
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def advance_scenario(name: str, *, slip: float = 0.0, zero_failure: bool = False):
    """Drive an independent plant through simulated accepted velocity output.

    The target is only passed to the production trajectory/controller. The
    simulated encoder independently samples physical state; the controller
    never receives plant.position or a copied target as its feedback.
    """
    clock = ModelClock()
    plant = MechanicalPlant(clock(), slip=slip)
    servo = SimServo(plant, clock)
    encoder = SimEncoder(plant, clock=clock, noise_mm=0.002, seed=7)
    estimator = LinearEstimator(0.01, 100.0, 0.2)
    controller = PositionController(kp=3.0, ki=0.0, kd=0.0)
    trajectory = Trapezoid(0.0, 10.0, 5.0, 20.0, clock())
    servo.open()
    servo.configure()
    servo.enable(True)
    encoder.open()
    rows = []
    previous_output, previous_sample_ns = 0.0, None
    accepted = requested = raw = 0.0
    reached_at = settled_since = stopped_at = None
    stop_requested_at = 0.8 if zero_failure else None
    period_ns, sample_stride = 10_000_000, 2
    last_estimate = None
    try:
        for step in range(801):
            clock.at_step(step, period_ns)
            time_s = clock() / 1e9
            physical = plant.advance(clock())
            reference = trajectory.at(clock())
            stop_requested = zero_failure and time_s >= stop_requested_at
            if step % sample_stride == 0:
                sample = encoder.read(clock() + 1_000_000)
                estimate = estimator.update(sample)
                if step == 0:
                    estimator.set_reference(0.0)
                    estimate = estimator.state
                last_estimate = estimate
                if estimate.velocity_valid:
                    dt = (
                        (sample.received_ns - previous_sample_ns) / 1e9
                        if previous_sample_ns is not None
                        else 0.02
                    )
                    previous_sample_ns = sample.received_ns
                    if stop_requested:
                        # The API returns without error, but the independent
                        # plant deliberately ignores the zero request. No
                        # successful write is used as stopping evidence.
                        plant.ignore_zero = True
                        servo.stop(clock() + 1_000_000)
                        requested = raw = 0.0
                        accepted = plant.snapshot().accepted_velocity_mm_s
                    elif reached_at is not None:
                        servo.stop(clock() + 1_000_000)
                        requested = raw = accepted = 0.0
                    else:
                        demand = controller.request(
                            reference.position,
                            reference.velocity,
                            estimate.position_mm,
                            estimate.velocity_mm_s,
                        )
                        raw = demand.requested
                        requested = slew(clamp(raw, -5.0, 5.0), previous_output, 20.0, dt)
                        requested = boundary_velocity(
                            requested, estimate.position_mm, -100.0, 100.0, 20.0, delay=0.07
                        )
                        accepted = servo.write_velocity(requested, clock() + 1_000_000)
                        controller.commit(
                            sample.seq,
                            1,
                            reference.position - estimate.position_mm,
                            raw,
                            requested,
                            dt,
                        )
                        controller.applied(sample.seq, 1, accepted)
                        previous_output = requested
                    at_target = (
                        reference.complete
                        and abs(10.0 - estimate.position_mm) <= 0.08
                        and abs(estimate.velocity_mm_s) <= 0.15
                    )
                    if at_target and not stop_requested:
                        if settled_since is None:
                            settled_since = clock()
                        if clock() - settled_since >= 150_000_000 and reached_at is None:
                            reached_at = time_s
                    else:
                        settled_since = None
            physical = plant.snapshot()
            if stop_requested and abs(physical.velocity_mm_s) <= 0.15 and stopped_at is None:
                stopped_at = time_s
            rows.append(
                {
                    "clock": "model_time",
                    "model_s": time_s,
                    "target_mm": 10.0,
                    "reference_mm": reference.position,
                    "encoder_mm": last_estimate.position_mm if last_estimate else 0.0,
                    "encoder_velocity_mm_s": last_estimate.velocity_mm_s if last_estimate else 0.0,
                    "plant_mm": physical.position_mm,
                    "plant_velocity_mm_s": physical.velocity_mm_s,
                    "raw_controller_mm_s": raw,
                    "requested_mm_s": requested,
                    "accepted_drive_mm_s": accepted,
                    "stop_requested": bool(stop_requested),
                    "stop_api_returned_success": bool(stop_requested),
                }
            )
            if not zero_failure and reached_at is not None and time_s >= reached_at + 0.5:
                break
        final = plant.snapshot()
        metrics = {
            "scenario": name,
            "clock": "model_time",
            "seed": 7,
            "scope": "production_algorithms_and_device_simulation_not_ROS_execution",
            "target_mm": 10.0,
            "slip_fraction": slip,
            "samples": len(rows),
            "final_error_mm": 10.0 - final.position_mm,
            "positive_overshoot_mm": max(0.0, max(row["plant_mm"] for row in rows) - 10.0),
            "reached_at_model_s": reached_at,
            "stop_requested_model_s": stop_requested_at,
            "stop_confirmed_from_model_velocity": stopped_at is not None,
            "stop_response_model_s": stopped_at - stop_requested_at
            if stopped_at is not None
            else None,
            "physical_validation": "NOT_RUN",
        }
        if zero_failure:
            metrics["expected_behavior"] = "zero_API_success_does_not_prove_stopped"
            metrics["passed"] = stopped_at is None and abs(final.velocity_mm_s) > 1.0
        else:
            metrics["expected_behavior"] = "closed_loop_reaches_target_from_encoder_feedback"
            metrics["passed"] = (
                reached_at is not None
                and abs(metrics["final_error_mm"]) <= 0.1
                and metrics["positive_overshoot_mm"] <= 0.5
            )
        return rows, metrics
    finally:
        plant.ignore_zero = False
        servo.close()
        encoder.close()


def rotation_scenario(name: str, *, missed_fraction: float = 0.0):
    """Exercise finite segments, complete receipts and an unobserved shaft model.

    This deterministic single-flight driver is a model harness, not a second
    runtime. The production Trapezoid/PulseQuantizer/SimPulseEngine own the
    motion mathematics and execution accounting being evaluated here.
    """
    clock = ModelClock()
    quantizer = PulseQuantizer(6400, 17 / 30)
    plant = MechanicalPlant(clock(), degrees_per_pulse=quantizer.angle(1))
    plant.missed_step_fraction = missed_fraction
    pulse = SimPulseEngine(plant, clock)
    identity = pulse.open()
    final_target = quantizer.pulses(30.0)
    trajectory = Trapezoid(0.0, quantizer.angle(final_target), 90.0, 360.0, clock())
    confirmed = reserved = submitted = 0
    rows = []
    reached_at = None
    period_ns = 5_000_000
    duration_s = 0.03
    try:
        for step in range(801):
            clock.at_step(step, period_ns)
            for completion in pulse.poll():
                confirmed += completion.signed_pulses
                reserved = 0
            reference = trajectory.at(clock())
            future = trajectory.at(clock() + round(duration_s * 1e9))
            planned = quantizer.pulses(future.position)
            if pulse.is_idle() and planned != confirmed:
                delta = planned - confirmed
                submitted += 1
                pulse.submit(
                    PulseSegment(
                        f"{name}-{submitted}", delta, duration_s, 1, identity.device_epoch, name
                    )
                )
                reserved = delta
            if reference.complete and confirmed == final_target and pulse.is_idle():
                reached_at = clock() / 1e9
            rows.append(
                {
                    "clock": "model_time",
                    "model_s": clock() / 1e9,
                    "requested_angle_deg": 30.0,
                    "target_pulses": final_target,
                    "planned_pulses": planned,
                    "reserved_pulses": reserved,
                    "confirmed_pulses": confirmed,
                    "confirmed_estimated_angle_deg": quantizer.angle(confirmed),
                    "independent_mechanical_angle_deg": plant.snapshot().mechanical_angle_deg,
                    "mechanical_angle_is_control_feedback": False,
                }
            )
            if reached_at is not None:
                break
        mechanical = plant.snapshot().mechanical_angle_deg
        estimate = quantizer.angle(confirmed)
        error = estimate - mechanical
        metrics = {
            "scenario": name,
            "clock": "model_time",
            "samples": len(rows),
            "scope": "finite_segment_algorithm_model_not_ROS_execution",
            "missed_step_fraction": missed_fraction,
            "target_pulses": final_target,
            "confirmed_pulses": confirmed,
            "confirmed_estimated_angle_deg": estimate,
            "independent_mechanical_angle_deg": mechanical,
            "open_loop_angle_error_deg": error,
            "quantized_target_error_deg": 30.0 - estimate,
            "reached_at_model_s": reached_at,
            "software_can_detect_unobservable_missed_steps": False,
            "physical_validation": "NOT_RUN",
            "passed": confirmed == final_target
            and pulse.is_idle()
            and (abs(error) > 5.0 if missed_fraction else math.isclose(error, 0.0, abs_tol=1e-9)),
        }
        return rows, metrics
    finally:
        pulse.close()


def _plot(path: Path, rows: list[dict], rotation: bool):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    time_s = [row["model_s"] for row in rows]
    if rotation:
        for key, label in (
            ("requested_angle_deg", "Requested angle"),
            ("confirmed_estimated_angle_deg", "Pulse estimate"),
            ("independent_mechanical_angle_deg", "Independent shaft model"),
        ):
            axes[0].plot(time_s, [row[key] for row in rows], label=label)
        for key in ("planned_pulses", "reserved_pulses", "confirmed_pulses"):
            axes[1].plot(time_s, [row[key] for row in rows], label=key)
        axes[0].set_ylabel("Angle (deg)")
        axes[1].set_ylabel("Pulse edges")
    else:
        for key, label in (
            ("target_mm", "Target"),
            ("reference_mm", "Trajectory"),
            ("encoder_mm", "Encoder"),
            ("plant_mm", "Independent plant"),
        ):
            axes[0].plot(time_s, [row[key] for row in rows], label=label)
        for key in ("requested_mm_s", "accepted_drive_mm_s", "plant_velocity_mm_s"):
            axes[1].plot(time_s, [row[key] for row in rows], label=key)
        axes[0].set_ylabel("Position (mm)")
        axes[1].set_ylabel("Velocity (mm/s)")
    for axis in axes:
        axis.grid(alpha=0.2)
        axis.legend(fontsize=8)
    axes[1].set_xlabel("Model time (s); not ROS wall time or hardware measurement")
    fig.suptitle(path.stem)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def run(output: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    scenarios = (
        ("advance_normal", lambda: advance_scenario("advance_normal"), False),
        ("advance_slip", lambda: advance_scenario("advance_slip", slip=0.25), False),
        (
            "zero_write_failure",
            lambda: advance_scenario("zero_write_failure", zero_failure=True),
            False,
        ),
        ("rotation_normal", lambda: rotation_scenario("rotation_normal"), True),
        (
            "rotation_missed_steps",
            lambda: rotation_scenario("rotation_missed_steps", missed_fraction=0.2),
            True,
        ),
    )
    results = []
    for name, simulate, rotation in scenarios:
        rows, metrics = simulate()
        _save_csv(output / f"{name}.csv", rows)
        _plot(output / f"{name}.png", rows, rotation)
        (output / f"{name}.json").write_text(
            json.dumps(metrics, indent=2, allow_nan=False) + "\n", encoding="utf-8"
        )
        results.append(metrics)
    summary = {
        "clock": "model_time",
        "deterministic": True,
        "seed": 7,
        "model_step_s": 0.01,
        "encoder_sample_period_s": 0.02,
        "rotation_model_step_s": 0.005,
        "model_assumptions": {
            "servo_tau_s": 0.04,
            "encoder_mm_per_count": 0.01,
            "encoder_noise_std_mm": 0.002,
            "controller_kp": 3.0,
            "controller_ki": 0.0,
            "controller_kd": 0.0,
            "limits_mm_s": 5.0,
            "limits_mm_s2": 20.0,
            "pulses_per_motor_rev": 6400,
            "catheter_per_motor_ratio": 17 / 30,
        },
        "scope": "deterministic_algorithm_and_independent_plant_report",
        "formal_ROS_path_tested_here": False,
        "hardware_tested": False,
        "passed": all(item["passed"] for item in results),
        "scenarios": results,
    }
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    return summary


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        summary = run(args.output)
        print(
            json.dumps(
                {
                    "passed": summary["passed"],
                    "clock": "model_time",
                    "scenarios": len(summary["scenarios"]),
                    "summary": str(args.output / "summary.json"),
                },
                allow_nan=False,
            )
        )
        return 0 if summary["passed"] else 1
    except Exception:
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
