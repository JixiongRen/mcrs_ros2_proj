"""Read-only identity, mapping and wireless state-contract evidence tools.

This tool never ARM/claims a session, writes input calibration, sends haptics,
opens motor ports or modifies permissions. Interactive physical work is always
explicit and bounded. Operator statements are marked as such, never inferred
from static metadata. Software-generated records cannot qualify real hardware.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from dataclasses import asdict, replace
from pathlib import Path
from time import monotonic_ns
from uuid import uuid4

from .config import (
    AxisBinding,
    MappingConfig,
    TriggerBinding,
    finite,
    load_profile,
    validate_mapping_evidence,
    validate_qualification,
)
from .input.joydev import NativeJoydev, discover
from .mapping import map_input
from .model import InputError


def _json_bytes(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":")).encode()


def _frame_record(frame):
    return {key: value for key, value in asdict(frame).items() if key != "identity"}


def _real_frames(frames, identity):
    result = []
    for frame in frames:
        if (
            frame.synthetic is not False
            or frame.identity is None
            or frame.identity.fingerprint != identity.fingerprint
            or frame.initialized is not True
            or frame.resync
            or frame.source_kind != "JOYDEV"
        ):
            raise ValueError("Only initialized physical frames of the exact identity are evidence")
        if frame.event_seq > 0 and (frame.changed_axes or frame.changed_buttons):
            result.append(frame)
    if not result:
        raise ValueError("No physical change events; INIT/cache/metadata are not mapping evidence")
    return result


class MappingWizard:
    """Pure recorder for explicitly named operator actions, no assumed Xbox map."""

    LABELS = (
        "lx_left",
        "lx_center",
        "lx_right",
        "ly_up",
        "ly_center",
        "ly_down",
        "lt_released",
        "lt_pressed",
        "b_pressed",
        "b_released",
        "rt_pressed",
        "rb_pressed",
        "rx_right",
        "ry_up",
        "rs_pressed",
    )

    def __init__(self, identity):
        if not identity.device_mode or identity.path == "scripted":
            raise ValueError("Physical device mode must be explicitly recorded")
        self.identity, self.steps = identity, {}
        self.selected = {}

    def capture(self, label, frames):
        if label not in self.LABELS:
            raise ValueError("Unknown mapping step")
        if not isinstance(frames, (tuple, list)) or not 1 <= len(frames) <= 512:
            raise ValueError("Mapping step requires 1..512 bounded frames")
        self.steps[label] = tuple(_real_frames(frames, self.identity))

    def _dominant(self, candidates, purpose):
        # Twofold dominance rejects a truly diagonal/ambiguous instruction but
        # permits natural off-axis motion. It is an input calibration policy,
        # not proof that the user reached the mechanical travel limit.
        candidates.sort(key=lambda candidate: candidate[0], reverse=True)
        if (
            not candidates
            or candidates[0][0] < 8192
            or len(candidates) > 1
            and candidates[0][0] < 2 * candidates[1][0]
        ):
            raise ValueError(
                f"Ambiguous or insufficient travel for {purpose}; repeat the labeled action"
            )
        return candidates[0]

    def _axis(self, negative, center, positive, *, positive_label_is_positive=True):
        middle = self.steps[center][-1]
        common = set(
            i
            for label in (negative, positive)
            for frame in self.steps[label]
            for i in frame.changed_axes
        )
        candidates = []
        for i in common:
            low_frames = [f for f in self.steps[negative] if i in f.changed_axes]
            high_frames = [f for f in self.steps[positive] if i in f.changed_axes]
            if not low_frames or not high_frames or type(middle.axes[i]) is not int:
                continue
            low = max(
                low_frames,
                key=lambda f: (abs(f.axes[i] - middle.axes[i]), f.observed_ns, f.event_seq),
            )
            high = max(
                high_frames,
                key=lambda f: (abs(f.axes[i] - middle.axes[i]), f.observed_ns, f.event_seq),
            )
            if (low.axes[i] - middle.axes[i]) * (high.axes[i] - middle.axes[i]) >= 0:
                continue
            score = min(abs(low.axes[i] - middle.axes[i]), abs(high.axes[i] - middle.axes[i]))
            candidates.append((score, i, low, high))
        _, i, low, high = self._dominant(candidates, negative + "/" + positive)
        minimum, maximum = sorted((low.axes[i], high.axes[i]))
        # A center capture far off the midpoint is not repaired by inventing a
        # zero. Preserve its raw value and demand a new operator observation.
        if abs(middle.axes[i] - (minimum + maximum) / 2) > (maximum - minimum) * 0.20:
            raise ValueError(f"Center observation for {center} is not neutral; recapture")
        sign = 1 if high.axes[i] > low.axes[i] else -1
        if not positive_label_is_positive:
            sign = -sign
        for label, frame in ((negative, low), (center, middle), (positive, high)):
            self.selected[label] = dict(_frame_record(frame), axis=i)
        return AxisBinding(i, minimum, middle.axes[i], maximum, sign)

    def _button(self, pressed, released=None):
        candidates = set(
            i
            for frame in self.steps[pressed]
            for i in frame.changed_buttons
            if frame.buttons[i] == 1
        )
        if released is not None:
            candidates &= set(
                i
                for frame in self.steps[released]
                for i in frame.changed_buttons
                if frame.buttons[i] == 0
            )
        if released is None and len(candidates) > 1:
            # A preceding control may finish releasing during this step. Prefer
            # the uniquely held requested button; multiple held candidates stay
            # ambiguous and a fully released multi-button trace cannot qualify.
            candidates &= {
                i for i, value in enumerate(self.steps[pressed][-1].buttons) if value == 1
            }
        if len(candidates) != 1:
            raise ValueError(f"One distinct observed button required for {pressed}")
        i = candidates.pop()
        self.selected[pressed] = dict(
            _frame_record(
                next(f for f in self.steps[pressed] if i in f.changed_buttons and f.buttons[i] == 1)
            ),
            button=i,
        )
        if released is not None:
            self.selected[released] = dict(
                _frame_record(
                    next(
                        f
                        for f in self.steps[released]
                        if i in f.changed_buttons and f.buttons[i] == 0
                    )
                ),
                button=i,
            )
        return i

    def build(self):
        if set(self.steps) != set(self.LABELS):
            raise ValueError("All mapping steps, including reserved controls, are required")
        self.selected = {}
        lx = self._axis("lx_left", "lx_center", "lx_right")
        ly = self._axis("ly_up", "ly_center", "ly_down", positive_label_is_positive=False)
        released = self.steps["lt_released"][-1]
        candidates = []
        for i in set(i for frame in self.steps["lt_pressed"] for i in frame.changed_axes):
            if i in (lx.index, ly.index) or not any(
                i in f.changed_axes for f in self.steps["lt_released"]
            ):
                continue
            frame = max(
                (f for f in self.steps["lt_pressed"] if i in f.changed_axes),
                key=lambda f: abs(f.axes[i] - released.axes[i]),
            )
            candidates.append((abs(frame.axes[i] - released.axes[i]), i, frame))
        _, i, pressed = self._dominant(candidates, "LT")
        release_label = "lt_released"
        release_direction = 1 if released.axes[i] > pressed.axes[i] else -1
        # A later explicitly recorded action may finish releasing LT. Refine
        # only along the already identified axis/direction, using an actual
        # change event, never an assumed zero or a fabricated full-scale value.
        release_samples = [(released, release_label)]
        release_samples.extend(
            (frame, label)
            for label, observed in self.steps.items()
            for frame in observed
            if i in frame.changed_axes
        )
        released, release_label = max(
            release_samples,
            key=lambda sample: (
                release_direction * sample[0].axes[i],
                sample[0].observed_ns,
                sample[0].event_seq,
                sample[1],
            ),
        )
        lt = TriggerBinding(i, released.axes[i], pressed.axes[i])
        self.selected["lt_released"] = dict(
            _frame_record(released), axis=i, observed_in=release_label
        )
        self.selected["lt_pressed"] = dict(_frame_record(pressed), axis=i)
        b_button = self._button("b_pressed", "b_released")
        reserved_axes, reserved_buttons = set(), set()
        for label in ("rt_pressed", "rx_right", "ry_up"):
            frames = self.steps[label]
            scores = []
            for axis in set(i for frame in frames for i in frame.changed_axes):
                if axis in {lx.index, ly.index, lt.index} | reserved_axes:
                    continue
                values = [f.axes[axis] for f in frames]
                # A one-event step has no starting sample; the actual observed
                # endpoint magnitude remains enough to identify a reserved axis.
                score = max(values) - min(values) if len(set(values)) > 1 else abs(values[0])
                scores.append((score, axis))
            _, axis = self._dominant(scores, label)
            reserved_axes.add(axis)
            self.selected[label] = dict(
                _frame_record(max(frames, key=lambda f: abs(f.axes[axis] - frames[0].axes[axis]))),
                axis=axis,
            )
        for label in ("rb_pressed", "rs_pressed"):
            reserved_buttons.add(self._button(label))
        if len(reserved_axes) != 3 or len(reserved_buttons) != 2:
            raise ValueError("Reserved controls must have distinct observations")
        mapping = MappingConfig(
            lx, ly, lt, b_button, tuple(sorted(reserved_axes)), tuple(sorted(reserved_buttons))
        )
        controls = {}
        groups = {
            "lx": ("lx_left", "lx_center", "lx_right"),
            "ly": ("ly_up", "ly_center", "ly_down"),
            "lt": ("lt_released", "lt_pressed"),
            "b": ("b_pressed", "b_released"),
        }
        for key, labels in groups.items():
            controls[key] = {
                "samples": [
                    dict(_frame_record(frame), real_event=True, label=label)
                    for label in labels
                    for frame in self.steps[label]
                ]
            }
        records = {
            label: [_frame_record(f) for f in frames] for label, frames in self.steps.items()
        }
        evidence = dict(
            schema_version=2,
            status="PASS",
            source_kind="JOYDEV_READONLY",
            record_id=uuid4().hex,
            identity=asdict(self.identity),
            identity_fingerprint=self.identity.fingerprint,
            mapping_fingerprint=mapping.fingerprint,
            mapping=asdict(mapping),
            controls=controls,
            selected_samples=self.selected,
            selection_policy="TRACE_PEAK_DOMINANCE_2_MIN_8192_CENTER_20PCT",
            raw_records=records,
            raw_records_sha256=hashlib.sha256(_json_bytes(records)).hexdigest(),
        )
        return mapping, evidence


class QualificationWizard:
    CASES = ("power_off", "rf_loss", "receiver_unplug")

    def __init__(self, profile, identity, max_disconnect_s):
        if profile.source != "joydev":
            raise ValueError("Physical joydev qualification only")
        validate_mapping_evidence(profile, identity)
        from .config import known_device_mode

        if not known_device_mode(identity.device_mode):
            raise ValueError("Known device mode required for qualification")
        if not identity.driver or not identity.kernel:
            raise ValueError("Observed driver/kernel required for qualification")
        finite(max_disconnect_s, "D", 0.001, 10.0)
        self.profile, self.identity, self.maximum = profile, identity, max_disconnect_s
        self.trials = []

    def add_trial(self, case, held_frame, start_ns, frames, final_health, *, operator_attested):
        if len(self.trials) >= 128:
            raise ValueError("Qualification trial capacity exceeded")
        trial = evaluate_qualification_trial(
            self.profile,
            self.identity,
            self.maximum,
            case,
            held_frame,
            start_ns,
            frames,
            final_health,
            operator_attested=operator_attested,
        )
        self.trials.append(trial)
        return trial

    def build(self):
        passed = all(
            len([t for t in self.trials if t["case"] == case and t["status"] == "PASS"]) >= 3
            for case in self.CASES
        ) and all(t["status"] == "PASS" for t in self.trials)
        result = dict(
            schema_version=2,
            status="PASS" if passed else "FAIL",
            source_kind="JOYDEV_READONLY",
            record_id=uuid4().hex,
            identity=asdict(self.identity),
            identity_fingerprint=self.identity.fingerprint,
            mapping_fingerprint=self.profile.mapping.fingerprint,
            device_mode=self.identity.device_mode,
            kernel=self.identity.kernel,
            max_disconnect_s=self.maximum,
            trials=self.trials,
            safety_note="D covers observed input response only; add software and actuator delays",
        )

        return validate_qualification_output(self.profile, self.identity, result)


def validate_qualification_output(profile, identity, result):
    """Writer and reader share one acceptance decision before any PASS output.

    JSON round-tripping validates the exact persistent representation, including
    raw-record hashes. Failure keeps the evidence but never writes a qualified
    profile or prints PASS for a certificate that the loader would reject.
    """
    if result["status"] != "PASS":
        return result
    try:
        validate_qualification(profile, identity, evidence=json.loads(_json_bytes(result)))
    except (OSError, ValueError, TypeError, KeyError, AttributeError, IndexError) as exc:
        result = dict(result, status="FAIL", validation_error=str(exc)[:512])
    return result


def evaluate_qualification_trial(
    profile,
    identity,
    maximum,
    case,
    held_frame,
    start_ns,
    frames,
    final_health,
    *,
    operator_attested,
):
    """Pure replay of held/failure/response evidence; no qualification file IO."""
    if not isinstance(frames, (tuple, list)) or len(frames) > 511:
        raise ValueError("Qualification response frame capacity exceeded")
    if case not in QualificationWizard.CASES or operator_attested is not True:
        raise ValueError("Operator must attest the actual case and its start")
    _real_frames([held_frame], identity)
    held = map_input(held_frame, profile)
    if held.lt < profile.lt_press or held.centered:
        raise ValueError("Trial must begin with LT and nonzero left stick held")
    if type(start_ns) is not int or not held_frame.observed_ns <= start_ns:
        raise ValueError("Invalid trial start")
    if (
        final_health.input_epoch != held_frame.input_epoch
        or final_health.source_id != held_frame.source_id
    ):
        raise ValueError("Trial health belongs to another source/epoch")
    for name in ("source_id", "input_epoch", "clock_domain_id"):
        value = getattr(held_frame, name)
        if not isinstance(value, str) or not 1 <= len(value) <= 128:
            raise ValueError("Invalid qualification source/epoch/clock provenance")
    if held_frame.event_seq < 1 or held_frame.observation_seq < 1:
        raise ValueError("Invalid qualification held-frame sequence")
    for name in ("event_seq", "observation_seq", "raw_event_ns", "observed_ns"):
        value = getattr(held_frame, name)
        if type(value) is not int or value < 0:
            raise ValueError("Invalid qualification held-frame time/sequence")
    if held_frame.raw_event_ns > held_frame.observed_ns:
        raise ValueError("Invalid qualification held-frame time")
    if (
        type(final_health.reader_heartbeat_ns) is not int
        or final_health.reader_heartbeat_ns < held_frame.observed_ns
    ):
        raise ValueError("Invalid qualification health time")
    response_ns, response_kind = None, ""
    records = [_frame_record(held_frame)]
    previous = held_frame
    for frame in frames:
        if (
            frame.synthetic is not False
            or frame.resync
            or frame.initialized is not True
            or frame.source_kind != "JOYDEV"
            or frame.clock_domain_id != held_frame.clock_domain_id
            or frame.input_epoch != held_frame.input_epoch
            or frame.source_id != held_frame.source_id
            or frame.identity is None
            or frame.identity.fingerprint != identity.fingerprint
        ):
            raise ValueError("Synthetic/reopened epochs cannot pass a trial")
        for name in ("event_seq", "observation_seq", "raw_event_ns", "observed_ns"):
            value = getattr(frame, name)
            if type(value) is not int or value < 0:
                raise ValueError("Invalid qualification frame time/sequence")
        if (
            frame.observation_seq <= previous.observation_seq
            or frame.observed_ns < previous.observed_ns
            or frame.raw_event_ns < previous.raw_event_ns
            or frame.raw_event_ns > frame.observed_ns
            or frame.event_seq < previous.event_seq
            or (
                frame.event_seq == previous.event_seq
                and (
                    frame.raw_event_ns != previous.raw_event_ns
                    or frame.axes != previous.axes
                    or frame.buttons != previous.buttons
                    or frame.changed_axes
                    or frame.changed_buttons
                )
            )
        ):
            raise ValueError("Qualification sequence/time replay")
        previous = frame
        # Cache observations are not physical events; retaining them in
        # full would inflate long D trials without adding link evidence.
        if frame.changed_axes or frame.changed_buttons:
            records.append(_frame_record(frame))
        # Arrival alone cannot establish causality: an old queued neutral may
        # be read after the fault cue. Require its conservative event-time lower
        # bound strictly after the cue, and an actual GCA-axis change. Ambiguous
        # near-boundary samples fail conservatively; timestamps are never moved.
        gca_axes = {profile.mapping.lx.index, profile.mapping.ly.index, profile.mapping.lt.index}
        if (
            frame.raw_event_ns <= start_ns
            or frame.observed_ns < start_ns
            or not set(frame.changed_axes) & gca_axes
        ):
            continue
        value = map_input(frame, profile)
        if value.lt <= profile.lt_release and value.centered:
            response_ns, response_kind = frame.observed_ns, "NEUTRAL_EVENT"
            break
    if final_health.reader_heartbeat_ns < previous.observed_ns:
        raise ValueError("Qualification health time precedes observed response")
    if not final_health.connected and final_health.reason == "DISCONNECTED":
        stamp = final_health.reader_heartbeat_ns
        if stamp >= start_ns and (response_ns is None or stamp < response_ns):
            response_ns, response_kind = stamp, "DISCONNECTED"
    passed = response_ns is not None and 0 <= response_ns - start_ns <= int(maximum * 1e9)
    raw = {"frames": records, "final_health": asdict(final_health)}
    trial = dict(
        trial_id=uuid4().hex,
        case=case,
        start_ns=start_ns,
        response_ns=response_ns,
        response_kind=response_kind,
        held_nonzero=True,
        operator_attested=True,
        status="PASS" if passed else "FAIL",
        raw_records=raw,
        raw_records_sha256=hashlib.sha256(_json_bytes(raw)).hexdigest(),
    )
    return trial


def _write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    # Refuse accidentally overwriting previous evidence. Records are auditable
    # local evidence, not signatures against an adversarial local administrator.
    if len(_json_bytes(value)) > 1_900_000:
        raise ValueError("Evidence exceeds loader capacity; reduce capture duration and repeat")
    with path.open("x", encoding="utf8") as stream:
        json.dump(value, stream, sort_keys=True, separators=(",", ":"))
        stream.write("\n")


def _write_profile(path, profile):
    import yaml

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    text = yaml.safe_dump(asdict(profile), sort_keys=False)
    if len(text.encode()) > 65_536:
        raise ValueError("Profile exceeds loader capacity")
    with path.open("x", encoding="utf8") as stream:
        stream.write("# Generated read-only input evidence; activation remains explicit.\n")
        stream.write(text)


def _collect(source, seconds):
    deadline = monotonic_ns() + int(seconds * 1e9)
    frames = []
    while monotonic_ns() < deadline:
        frames.extend(source.poll())
        if len(frames) > 10_000:
            raise InputError("PROBE_RECORD_BUDGET_EXCEEDED")
        if not source.health().connected:
            break
    return frames


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("inspect", "mapping", "qualify"))
    parser.add_argument("--profile", required=True)
    parser.add_argument(
        "--device-mode", help="Explicit operator-observed controller/receiver mode label"
    )
    parser.add_argument("--output")
    parser.add_argument(
        "--profile-output", help="Generated profile path; default OUTPUT.profile.yaml"
    )
    parser.add_argument("--max-disconnect-s", type=float)
    parser.add_argument("--capture-s", type=float, default=3.0)
    args = parser.parse_args(argv)
    profile = load_profile(args.profile)
    if args.device_mode is not None:
        profile = replace(profile, device=replace(profile.device, device_mode=args.device_mode))
    if profile.source != "joydev":
        parser.error("Physical probe requires a joydev profile")
    finite(args.capture_s, "capture duration", 0.05, 15)
    if args.command == "inspect":
        identity = discover(profile.device)
        result = dict(
            status="NOT_RUN",
            identity=asdict(identity),
            identity_fingerprint=identity.fingerprint,
            mapping="NOT_RUN",
            wireless_qualification="NOT_RUN",
        )
        print(json.dumps(result, indent=2))
        if args.output:
            _write(args.output, result)
        return 0
    if not args.output:
        parser.error("--output is required for evidence")
    source = NativeJoydev(profile)
    try:
        identity = source.open()
        _collect(source, 0.15)
        if args.command == "mapping":
            wizard = MappingWizard(identity)
            for label in wizard.LABELS:
                input(
                    f"{label}: press Enter, then perform ONLY this control action during capture: "
                )
                # Waiting for operator input is outside reader polling; reopen
                # gets a clean INIT baseline instead of replaying the old queue.
                source.close()
                source.open()
                _collect(source, 0.08)
                print(f"Capture active: perform {label} now.")
                wizard.capture(label, _collect(source, args.capture_s))
            mapping, evidence = wizard.build()
            _write(args.output, evidence)
            generated = replace(
                profile,
                mapping=mapping,
                mapping_evidence=str(Path(args.output).resolve()),
                mode="event_age",
                qualification_file="",
            )
            profile_path = args.profile_output or str(
                Path(args.output).with_suffix(".profile.yaml")
            )
            _write_profile(profile_path, generated)
            print(
                f"Generated inactive profile: {profile_path}. Load only after confirmed STOP and DISARM."
            )
            print("Observed mapping:")
            print(json.dumps(asdict(mapping), indent=2))
            return 0
        if args.max_disconnect_s is None:
            parser.error("Predeclare --max-disconnect-s before trials")
        wizard = QualificationWizard(profile, identity, args.max_disconnect_s)
        for case in wizard.CASES:
            for trial in range(3):
                input(
                    f"{case} trial {trial + 1}/3: reconnect, hold LT and nonzero left; Enter to sample: "
                )
                source.close()
                source.open()
                held_frames = _collect(source, 0.08)
                # Held baseline alone is not an actuation observation. Ask for
                # a small left movement while retaining LT, captured read-only.
                print("Keep LT held and move left stick slightly now.")
                held_frames.extend(_collect(source, args.capture_s))
                candidates = [f for f in held_frames if f.event_seq and f.changed_axes]
                if not candidates:
                    raise InputError("NO_HELD_PHYSICAL_EVENT")
                held = candidates[-1]
                # External action timestamp is operator attestation, not a
                # claim that software can observe the RF-loss onset itself.
                print(
                    f"Perform {case} NOW. Start is operator-attested local cue; keep holding controls."
                )
                start = monotonic_ns()
                frames = _collect(source, args.max_disconnect_s + 0.05)
                final_health = source.health()
                attested = (
                    input(
                        f"Type CONFIRM only if you performed {case} and kept LT+left held throughout: "
                    )
                    == "CONFIRM"
                )
                wizard.add_trial(
                    case, held, start, frames, final_health, operator_attested=attested
                )
        result = validate_qualification_output(profile, identity, wizard.build())
        _write(args.output, result)
        if result["status"] == "PASS":
            generated = replace(
                profile, mode="qualified_state", qualification_file=str(Path(args.output).resolve())
            )
            profile_path = args.profile_output or str(
                Path(args.output).with_suffix(".profile.yaml")
            )
            _write_profile(profile_path, generated)
            print(
                f"Generated inactive qualified profile: {profile_path}; explicit stopped/DISARM load required."
            )
        print(result["status"])
        return 0 if result["status"] == "PASS" else 1
    finally:
        source.close()


if __name__ == "__main__":
    raise SystemExit(main())
