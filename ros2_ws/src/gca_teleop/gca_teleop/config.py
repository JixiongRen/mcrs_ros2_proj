"""Versioned input profiles and evidence-backed state-contract qualification.

Synthetic defaults support repeatable simulation. Unknown physical mappings
remain null; a boolean cannot replace the required calibration/qualification
records. Loading files is a management operation, never an input tick action.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass, field, fields
from math import isfinite
from pathlib import Path


def finite(value, name, low=None, high=None):
    if type(value) not in (float, int) or not isfinite(value):
        raise ValueError(f"{name} must be finite numeric")
    if low is not None and value < low or high is not None and value > high:
        raise ValueError(f"{name} outside range")


def integer(value, name, low=0, high=65535):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f"{name} must be integer in [{low}, {high}]")


def bounded_string(value, name, *, required=True):
    if not isinstance(value, str) or len(value) > 512 or required and not value:
        raise ValueError(f"{name} must be a bounded string")


@dataclass(frozen=True)
class DeviceSelector:
    vendor: int | None = None
    product: int | None = None
    name: str | None = None
    path: str | None = None
    axis_codes: tuple[int, ...] | None = None
    button_codes: tuple[int, ...] | None = None
    device_mode: str = ""

    def __post_init__(self):
        for name in ("vendor", "product"):
            if getattr(self, name) is not None:
                integer(getattr(self, name), name)
        for name in ("name", "path"):
            if getattr(self, name) is not None:
                bounded_string(getattr(self, name), name)
        bounded_string(self.device_mode, "device_mode", required=False)
        for name in ("axis_codes", "button_codes"):
            values = getattr(self, name)
            if values is not None:
                if not isinstance(values, tuple) or not 1 <= len(values) <= 256:
                    raise ValueError(f"invalid {name}")
                for value in values:
                    integer(value, name)


@dataclass(frozen=True)
class AxisBinding:
    index: int
    minimum: float
    center: float
    maximum: float
    sign: int = 1

    def __post_init__(self):
        integer(self.index, "axis index", high=255)
        for name in ("minimum", "center", "maximum"):
            finite(getattr(self, name), name, -32768, 32767)
        if (
            not self.minimum < self.center < self.maximum
            or type(self.sign) is not int
            or self.sign not in (-1, 1)
        ):
            raise ValueError("invalid stick range/polarity")


@dataclass(frozen=True)
class TriggerBinding:
    index: int
    released: float
    pressed: float

    def __post_init__(self):
        integer(self.index, "trigger index", high=255)
        finite(self.released, "released", -32768, 32767)
        finite(self.pressed, "pressed", -32768, 32767)
        if self.released == self.pressed:
            raise ValueError("trigger needs distinct released/pressed observations")


@dataclass(frozen=True)
class MappingConfig:
    lx: AxisBinding | None = None
    ly: AxisBinding | None = None
    lt: TriggerBinding | None = None
    b_button: int | None = None
    reserved_axes: tuple[int, ...] = ()
    reserved_buttons: tuple[int, ...] = ()

    def __post_init__(self):
        for name, kind in (("lx", AxisBinding), ("ly", AxisBinding), ("lt", TriggerBinding)):
            if getattr(self, name) is not None and not isinstance(getattr(self, name), kind):
                raise ValueError(f"{name} has wrong binding type")
        if self.b_button is not None:
            integer(self.b_button, "B button", high=255)
        selected = [x.index for x in (self.lx, self.ly, self.lt) if x is not None]
        if len(selected) != len(set(selected)):
            raise ValueError("LX/LY/LT must use distinct axes")
        for values in (self.reserved_axes, self.reserved_buttons):
            if not isinstance(values, tuple) or len(set(values)) != len(values):
                raise ValueError("reserved controls must be unique tuples")
            for value in values:
                integer(value, "reserved control", high=255)
        if set(selected) & set(self.reserved_axes) or self.b_button in self.reserved_buttons:
            raise ValueError("reserved controls overlap GCA mapping")

    @property
    def complete(self):
        return all(x is not None for x in (self.lx, self.ly, self.lt, self.b_button))

    @property
    def fingerprint(self):
        return hashlib.sha256(json.dumps(asdict(self), sort_keys=True).encode()).hexdigest()


@dataclass(frozen=True)
class TeleopConfig:
    source: str = "joydev"
    profile_id: str = "unverified-input"
    mode: str = "event_age"
    device: DeviceSelector = field(default_factory=DeviceSelector)
    mapping: MappingConfig = field(default_factory=MappingConfig)
    mapping_evidence: str = ""
    qualification_file: str = ""
    script_file: str = ""
    dispatch_hz: float = 25.0
    reader_timeout_s: float = 0.10
    input_max_age_s: float = 0.15
    poll_timeout_s: float = 0.02
    max_poll_events: int = 64
    max_poll_bytes: int = 512
    timestamp_uncertainty_s: float = 0.020
    max_epoch_s: float = 3600.0
    deadzone: float = 0.08
    response_exponent: float = 1.5
    lt_press: float = 0.6
    lt_release: float = 0.4
    max_linear_mm_s: float = 2.0
    max_rotation_deg_s: float = 30.0
    seed: int = 7

    def __post_init__(self):
        if self.source not in ("joydev", "scripted") or self.mode not in (
            "event_age",
            "qualified_state",
        ):
            raise ValueError("unknown input source or freshness mode")
        for name in ("profile_id", "mapping_evidence", "qualification_file", "script_file"):
            bounded_string(getattr(self, name), name, required=name == "profile_id")
        if len(self.profile_id) > 96:
            raise ValueError("profile ID exceeds quality source ID capacity")
        if not isinstance(self.device, DeviceSelector) or not isinstance(
            self.mapping, MappingConfig
        ):
            raise ValueError("typed device/mapping required")
        for name in (
            "dispatch_hz",
            "reader_timeout_s",
            "input_max_age_s",
            "poll_timeout_s",
            "max_epoch_s",
            "response_exponent",
            "max_linear_mm_s",
            "max_rotation_deg_s",
        ):
            finite(getattr(self, name), name, 0.000001)
        for name in ("reader_timeout_s", "input_max_age_s", "poll_timeout_s"):
            finite(getattr(self, name), name, 0.000001, 3600)
        if self.max_epoch_s > 86400:
            raise ValueError("epoch bound must remain below u32 half-wrap")
        if self.dispatch_hz > 250 or self.poll_timeout_s >= self.reader_timeout_s:
            raise ValueError("invalid reader/dispatch rate")
        finite(self.timestamp_uncertainty_s, "timestamp uncertainty", 0.0, self.input_max_age_s / 2)
        finite(self.deadzone, "deadzone", 0.0, 0.9)
        finite(self.lt_release, "LT release", 0.0, 1.0)
        finite(self.lt_press, "LT press", 0.0, 1.0)
        if self.lt_release >= self.lt_press:
            raise ValueError("LT press threshold must exceed release threshold")
        integer(self.max_poll_events, "max_poll_events", 1, 256)
        integer(self.max_poll_bytes, "max_poll_bytes", 8, 4096)
        if self.max_poll_bytes % 8:
            raise ValueError("read byte limit must contain complete event slots")
        integer(self.seed, "seed", 0, 2147483647)

    @property
    def fingerprint(self):
        return hashlib.sha256(json.dumps(asdict(self), sort_keys=True).encode()).hexdigest()


Profile = TeleopConfig


def _typed(cls, data):
    if not isinstance(data, dict):
        raise ValueError(f"{cls.__name__} must be mapping")
    unknown = set(data) - {f.name for f in fields(cls)}
    if unknown:
        raise ValueError(f"unknown {cls.__name__} keys: {sorted(unknown)}")
    return cls(**data)


def profile_from_dict(data):
    if not isinstance(data, dict):
        raise ValueError("profile must be mapping")
    data = dict(data)
    device = dict(data.get("device", {}))
    for name in ("axis_codes", "button_codes"):
        if device.get(name) is not None:
            device[name] = tuple(device[name])
    data["device"] = _typed(DeviceSelector, device)
    mapping = dict(data.get("mapping", {}))
    for name, cls in (("lx", AxisBinding), ("ly", AxisBinding), ("lt", TriggerBinding)):
        if mapping.get(name) is not None:
            mapping[name] = _typed(cls, mapping[name])
    for name in ("reserved_axes", "reserved_buttons"):
        if name in mapping:
            mapping[name] = tuple(mapping[name])
    data["mapping"] = _typed(MappingConfig, mapping)
    return _typed(TeleopConfig, data)


def bounded_text(path, limit):
    # Bound the actual read, not only a racy pre-read stat size.
    with Path(path).open("rb") as stream:
        value = stream.read(limit + 1)
    if len(value) > limit:
        raise ValueError("Input file exceeds size limit")
    return value.decode("utf8")


def load_profile(path):
    import yaml

    path = Path(path).resolve()
    try:
        document = yaml.safe_load(bounded_text(path, 65_536))
    except RecursionError:
        raise ValueError("Profile nesting exceeds limit") from None
    result = profile_from_dict(document)
    replacements = {}
    for name in ("mapping_evidence", "qualification_file", "script_file"):
        value = getattr(result, name)
        if value:
            replacements[name] = str((path.parent / value).resolve())
    from dataclasses import replace

    return replace(result, **replacements)


def read_evidence(path):
    try:
        value = json.loads(bounded_text(path, 2_000_000))
    except RecursionError:
        raise ValueError("Evidence nesting exceeds limit") from None
    if not isinstance(value, dict):
        raise ValueError("evidence must be JSON object")
    return value


def validate_mapping_evidence(profile, identity, *, evidence=None):
    if not profile.mapping.complete:
        raise ValueError("MAPPING_UNVERIFIED")
    if profile.source == "scripted":
        return "SYNTHETIC_PROFILE"
    data = read_evidence(profile.mapping_evidence) if evidence is None else evidence
    if data.get("schema_version") == 3:
        from .mode_confirmation import validate_mode_confirmation

        return validate_mode_confirmation(profile, identity, data)
    if (
        data.get("schema_version") != 2
        or data.get("status") != "PASS"
        or data.get("source_kind") != "JOYDEV_READONLY"
        or data.get("identity_fingerprint") != identity.fingerprint
        or data.get("mapping_fingerprint") != profile.mapping.fingerprint
    ):
        raise ValueError("MAPPING_EVIDENCE_MISMATCH")
    required = {"lx", "ly", "lt", "b"}
    controls = data.get("controls", {})
    if not isinstance(controls, dict) or not required <= set(controls):
        raise ValueError("MAPPING_EVIDENCE_INCOMPLETE")
    for key in required:
        if not isinstance(controls[key], dict):
            raise ValueError("MAPPING_EVIDENCE_INCOMPLETE")
        samples = controls[key].get("samples", [])
        if (
            not isinstance(samples, list)
            or len(samples) < 2
            or not all(
                isinstance(s, dict)
                and s.get("real_event") is True
                and s.get("synthetic") is False
                and s.get("event_seq", 0) > 0
                and (s.get("changed_axes") or s.get("changed_buttons"))
                for s in samples
            )
        ):
            raise ValueError("MAPPING_EVIDENCE_NO_EVENTS")
    _check_records(data)
    # Recompute the selection from the exact bounded raw observations. Hashes
    # detect corruption; they do not prove that a claimed mapping used them.
    from .probe import MappingWizard

    wizard = MappingWizard(identity)
    for label, records in data["raw_records"].items():
        wizard.capture(label, _restore_frames(records, identity))
    mapping, rebuilt = wizard.build()
    for name in ("mapping", "controls", "selected_samples", "selection_policy"):
        if _canonical(data.get(name)) != _canonical(rebuilt[name]):
            raise ValueError("MAPPING_RAW_RECORDS_MISMATCH")
    if mapping.fingerprint != profile.mapping.fingerprint:
        raise ValueError("MAPPING_RAW_RECORDS_MISMATCH")
    record_id = data.get("record_id", "")
    bounded_string(record_id, "mapping ID")
    return record_id


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def _restore_frames(records, identity):
    from .model import InputFrame

    if not isinstance(records, (tuple, list)) or not 1 <= len(records) <= 512:
        raise ValueError("EVIDENCE_FRAME_CAPACITY")
    frames = []
    for raw in records:
        if not isinstance(raw, dict):
            raise ValueError("INVALID_EVIDENCE_FRAME")
        raw = dict(raw)
        for name in ("axes", "buttons", "changed_axes", "changed_buttons"):
            values = raw.get(name, [])
            if not isinstance(values, (tuple, list)) or len(values) > 256:
                raise ValueError("INVALID_EVIDENCE_FRAME")
            raw[name] = tuple(values)
        for name in ("event_seq", "raw_event_ns", "observation_seq", "observed_ns"):
            integer(raw.get(name), name, 0, 2**64 - 1)
        if raw["raw_event_ns"] > raw["observed_ns"]:
            raise ValueError("INVALID_EVIDENCE_FRAME_TIME")
        if len(raw["axes"]) != len(identity.axis_codes) or len(raw["buttons"]) != len(
            identity.button_codes
        ):
            raise ValueError("EVIDENCE_CAPABILITY_MISMATCH")
        for value in raw["axes"]:
            integer(value, "observed axis", -32768, 32767)
        for value in raw["buttons"]:
            integer(value, "observed button", 0, 1)
        for index in raw["changed_axes"]:
            integer(index, "changed axis", 0, len(raw["axes"]) - 1)
        for index in raw["changed_buttons"]:
            integer(index, "changed button", 0, len(raw["buttons"]) - 1)
        frames.append(InputFrame(**raw, identity=identity))
    return frames


def known_device_mode(value):
    return (
        isinstance(value, str)
        and bool(value)
        and not any(
            marker in value.casefold()
            for marker in ("unknown", "unverified", "not_run", "未知", "未确认")
        )
    )


def _check_records(record):
    records = record.get("raw_records")
    digest = record.get("raw_records_sha256")
    if not isinstance(records, dict) or not isinstance(digest, str):
        raise ValueError("EVIDENCE_RAW_RECORDS_REQUIRED")
    actual = hashlib.sha256(
        json.dumps(records, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    if digest != actual:
        raise ValueError("EVIDENCE_HASH_MISMATCH")


def validate_qualification(profile, identity, *, evidence=None, mapping_evidence=None):
    """Validate recorded state-contract evidence, not current RF connectivity."""
    validate_mapping_evidence(profile, identity, evidence=mapping_evidence)
    data = read_evidence(profile.qualification_file) if evidence is None else evidence
    if (
        data.get("schema_version") != 2
        or data.get("status") != "PASS"
        or data.get("source_kind") != "JOYDEV_READONLY"
        or data.get("identity_fingerprint") != identity.fingerprint
        or data.get("mapping_fingerprint") != profile.mapping.fingerprint
        or not known_device_mode(identity.device_mode)
        or not identity.driver
        or not identity.kernel
        or data.get("device_mode") != identity.device_mode
        or data.get("kernel") != identity.kernel
    ):
        raise ValueError("QUALIFICATION_MISMATCH")
    maximum = data.get("max_disconnect_s")
    finite(maximum, "qualification D", 0.001, 10.0)
    cases = {"power_off", "rf_loss", "receiver_unplug"}
    trials = data.get("trials", [])
    if not isinstance(trials, list) or len(trials) > 128:
        raise ValueError("QUALIFICATION_INCOMPLETE")
    seen_ids, seen_records = set(), set()
    for case in cases:
        matches = [t for t in trials if isinstance(t, dict) and t.get("case") == case]
        if len(matches) < 3:
            raise ValueError("QUALIFICATION_INCOMPLETE")
        for trial in matches:
            trial_id = trial.get("trial_id", "")
            bounded_string(trial_id, "trial ID")
            if trial_id in seen_ids or trial.get("raw_records_sha256") in seen_records:
                raise ValueError("QUALIFICATION_DUPLICATE_TRIAL")
            seen_ids.add(trial_id)
            seen_records.add(trial.get("raw_records_sha256"))
            _check_records(trial)
            if trial.get("status") != "PASS":
                raise ValueError("QUALIFICATION_TRIAL_FAILED")
            raw = trial["raw_records"]
            if not isinstance(raw.get("frames"), list) or not raw["frames"]:
                raise ValueError("QUALIFICATION_RAW_EVENTS_REQUIRED")
            if any(
                not isinstance(f, dict) or f.get("synthetic") is not False for f in raw["frames"]
            ):
                raise ValueError("QUALIFICATION_SYNTHETIC_FORBIDDEN")
            start, end = trial.get("start_ns"), trial.get("response_ns")
            integer(start, "trial start", 1, 2**63 - 1)
            integer(end, "trial response", start, 2**63 - 1)
            if (
                trial.get("held_nonzero") is not True
                or trial.get("operator_attested") is not True
                or trial.get("response_kind") not in ("NEUTRAL_EVENT", "DISCONNECTED")
                or (end - start) / 1e9 > maximum
                or not trial.get("raw_records_sha256")
            ):
                raise ValueError("QUALIFICATION_TRIAL_FAILED")
            # Re-evaluate claimed response from actual held/response records;
            # a PASS bit and a valid hash alone cannot manufacture a neutral.
            from .model import InputHealth, InputIdentity
            from .probe import evaluate_qualification_trial

            frames = _restore_frames(raw["frames"], identity)
            health_data = dict(raw.get("final_health", {}))
            recorded_identity = health_data.pop("identity", None)
            if not isinstance(recorded_identity, dict):
                raise ValueError("QUALIFICATION_IDENTITY_REQUIRED")
            for name in ("axis_codes", "button_codes"):
                recorded_identity[name] = tuple(recorded_identity.get(name, ()))
            health_identity = InputIdentity(**recorded_identity)
            if health_identity.fingerprint != identity.fingerprint:
                raise ValueError("QUALIFICATION_IDENTITY_MISMATCH")
            health = InputHealth(**health_data, identity=health_identity)
            checked = evaluate_qualification_trial(
                profile,
                identity,
                maximum,
                case,
                frames[0],
                start,
                frames[1:],
                health,
                operator_attested=trial["operator_attested"],
            )
            if any(
                checked[name] != trial[name]
                for name in ("status", "response_ns", "response_kind", "held_nonzero")
            ):
                raise ValueError("QUALIFICATION_RAW_RESPONSE_MISMATCH")
    record_id = data.get("record_id", "")
    bounded_string(record_id, "qualification ID")
    if len(record_id) > 128:
        raise ValueError("Qualification ID exceeds quality capacity")
    return record_id
