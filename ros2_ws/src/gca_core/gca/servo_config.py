"""Strict, ROS-independent manual and external-position servo configuration.

The editable values live in ``config/servo_settings.py``.  This module defines
the shared schema used by that exporter and installed consumers.  Loading a
configuration never imports an SDK or opens a device.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import asdict, dataclass, fields
from pathlib import Path

from gca.domain.position import MechanicalMapping, ResolvedBraking, resolve_braking_parameters
from gca.domain.servo import MANUAL_SERVO_VELOCITY, POSITION_EXTERNAL_ENCODER


def _number(value, name, *, positive=True):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"{name} must be finite numeric")
    if (positive and value <= 0) or (not positive and value < 0):
        raise ValueError(f"{name} outside its positive/nonnegative range")


def _integer(value, name, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f"{name} must be integer in [{low}, {high}]")


def _text(value, name, limit=128):
    if type(value) is not str or not value or len(value) > limit:
        raise ValueError(f"{name} must be a nonempty string of at most {limit} characters")


@dataclass(frozen=True)
class NativeDynamixelConfig:
    port: str
    baudrate: int
    device_id: int
    model_number: int
    velocity_unit_rpm: float
    max_velocity_raw: int
    watchdog_raw: int
    profile_acceleration_raw: int | None
    transaction_timeout_s: float

    def __post_init__(self):
        _text(self.port, "port", 512)
        _integer(self.baudrate, "baudrate", 9600, 4_000_000)
        _integer(self.device_id, "device_id", 0, 252)
        _integer(self.model_number, "model_number", 1, 65535)
        _number(self.velocity_unit_rpm, "velocity_unit_rpm")
        if self.velocity_unit_rpm != 0.229:
            raise ValueError("XM430-W350 velocity unit must be 0.229 rpm/raw")
        _integer(self.max_velocity_raw, "max_velocity_raw", 1, 1023)
        _integer(self.watchdog_raw, "watchdog_raw", 1, 127)
        if self.profile_acceleration_raw is not None:
            _integer(self.profile_acceleration_raw, "profile_acceleration_raw", 1, 32767)
        _number(self.transaction_timeout_s, "transaction_timeout_s")

    @property
    def max_rpm(self):
        return self.max_velocity_raw * self.velocity_unit_rpm


@dataclass(frozen=True)
class PositionMechanicalSettings:
    drive_wheel_radius_mm: float
    shaft_revolutions_per_wheel_revolution: float
    mm_per_shaft_revolution: float
    direction_sign: int
    calibration_id: str

    def __post_init__(self):
        for name in (
            "drive_wheel_radius_mm",
            "shaft_revolutions_per_wheel_revolution",
            "mm_per_shaft_revolution",
        ):
            _number(getattr(self, name), name)
        if type(self.direction_sign) is not int or self.direction_sign not in (-1, 1):
            raise ValueError("position direction_sign must be -1 or 1")
        _text(self.calibration_id, "calibration_id")
        if self.calibration_id.startswith("geometry") and not math.isclose(
            self.mm_per_shaft_revolution,
            2 * math.pi * self.drive_wheel_radius_mm / self.shaft_revolutions_per_wheel_revolution,
            rel_tol=1e-9,
        ):
            raise ValueError("servo geometry scale mismatch")

    @property
    def mapping(self):
        return MechanicalMapping(
            self.mm_per_shaft_revolution, self.direction_sign, self.calibration_id
        )


@dataclass(frozen=True)
class PositionPIDSettings:
    kp: float
    ki: float
    kd: float
    kaw: float
    integral_limit_mm_s: float
    dt_max_s: float

    def __post_init__(self):
        for item in fields(self):
            _number(getattr(self, item.name), item.name, positive=item.name == "dt_max_s")


@dataclass(frozen=True)
class PositionMotionSettings:
    linear_min_mm: float
    linear_max_mm: float
    max_linear_mm_s: float
    max_linear_mm_s2: float
    braking_deceleration_mm_s2: float | None
    braking_delay_s: float | None
    boundary_margin_mm: float
    position_tolerance_mm: float
    velocity_tolerance_mm_s: float
    settle_s: float

    def __post_init__(self):
        for name in ("linear_min_mm", "linear_max_mm"):
            value = getattr(self, name)
            if type(value) not in (float, int) or not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        for name in (
            "max_linear_mm_s",
            "max_linear_mm_s2",
            "position_tolerance_mm",
            "velocity_tolerance_mm_s",
            "settle_s",
        ):
            _number(getattr(self, name), name)
        _number(self.boundary_margin_mm, "boundary_margin_mm", positive=False)
        if self.target_min_mm >= self.target_max_mm:
            raise ValueError("position bounds leave no usable travel")
        if self.velocity_tolerance_mm_s >= self.max_linear_mm_s:
            raise ValueError("position velocity tolerance must be below motion speed")
        for name in ("braking_deceleration_mm_s2", "braking_delay_s"):
            if getattr(self, name) is not None:
                _number(getattr(self, name), name, positive=name != "braking_delay_s")

    @property
    def target_min_mm(self):
        return self.linear_min_mm + self.boundary_margin_mm

    @property
    def target_max_mm(self):
        return self.linear_max_mm - self.boundary_margin_mm


@dataclass(frozen=True)
class PositionBrakingDefaults:
    braking_deceleration_mm_s2: float
    braking_delay_s: float

    def __post_init__(self):
        _number(self.braking_deceleration_mm_s2, "braking_deceleration_mm_s2")
        _number(self.braking_delay_s, "braking_delay_s", positive=False)


@dataclass(frozen=True)
class PositionEncoderSettings:
    model: str
    port: str
    baudrate: int
    bytesize: int
    parity: str
    stopbits: int
    address: int
    function_code: int
    register_start: int
    register_count: int
    bits: int
    signed: bool
    byte_order_in_register: str
    word_order: str
    crc_byte_order: str
    count_modulus: int
    counts_per_revolution: int
    measurement_wheel_radius_mm: float
    encoder_revolutions_per_wheel_revolution: float
    mm_per_count: float
    direction_sign: int
    calibration_id: str
    sample_period_s: float
    feedback_max_age_s: float
    transaction_timeout_s: float
    quiet_s: float
    max_speed_mm_s: float
    velocity_window_s: float

    def __post_init__(self):
        _text(self.model, "encoder.model")
        _text(self.port, "encoder.port", 512)
        _text(self.calibration_id, "encoder.calibration_id")
        _integer(self.baudrate, "encoder.baudrate", 9600, 115200)
        _integer(self.address, "encoder.address", 1, 247)
        for name, expected in (
            ("bytesize", 8),
            ("stopbits", 1),
            ("function_code", 3),
            ("register_start", 0),
            ("register_count", 2),
            ("bits", 32),
        ):
            _integer(getattr(self, name), f"encoder.{name}", expected, expected)
        if self.parity != "N" or type(self.signed) is not bool or self.signed:
            raise ValueError("position encoder requires 8N1 unsigned counts")
        if (self.byte_order_in_register, self.word_order, self.crc_byte_order) != (
            "big",
            "high_word_first",
            "little",
        ):
            raise ValueError("unsupported encoder frame/count ordering")
        _integer(self.count_modulus, "count_modulus", 2, 2**32)
        _integer(self.counts_per_revolution, "counts_per_revolution", 2, 2**20)
        if type(self.direction_sign) is not int or self.direction_sign not in (-1, 1):
            raise ValueError("encoder direction_sign must be -1 or 1")
        for name in (
            "measurement_wheel_radius_mm",
            "encoder_revolutions_per_wheel_revolution",
            "mm_per_count",
            "sample_period_s",
            "feedback_max_age_s",
            "transaction_timeout_s",
            "quiet_s",
            "max_speed_mm_s",
            "velocity_window_s",
        ):
            _number(getattr(self, name), f"encoder.{name}")
        if self.sample_period_s + self.transaction_timeout_s >= self.feedback_max_age_s:
            raise ValueError("encoder period plus transaction must fit feedback age")
        if not self.sample_period_s < self.velocity_window_s <= 2.0:
            raise ValueError("encoder velocity window must be bounded and exceed sample period")
        if self.calibration_id.startswith("geometry") and not math.isclose(
            self.mm_per_count,
            2
            * math.pi
            * self.measurement_wheel_radius_mm
            / (self.encoder_revolutions_per_wheel_revolution * self.counts_per_revolution),
            rel_tol=1e-9,
        ):
            raise ValueError("encoder geometry scale mismatch")


@dataclass(frozen=True)
class PositionSettings:
    mechanics: PositionMechanicalSettings
    pid: PositionPIDSettings
    motion: PositionMotionSettings
    braking_defaults: PositionBrakingDefaults
    effective_braking: ResolvedBraking
    encoder: PositionEncoderSettings

    def __post_init__(self):
        for name, cls in (
            ("mechanics", PositionMechanicalSettings),
            ("pid", PositionPIDSettings),
            ("motion", PositionMotionSettings),
            ("braking_defaults", PositionBrakingDefaults),
            ("effective_braking", ResolvedBraking),
            ("encoder", PositionEncoderSettings),
        ):
            if type(getattr(self, name)) is not cls:
                raise ValueError(f"position.{name} must be {cls.__name__}")
        resolved = resolve_braking_parameters(asdict(self.motion), asdict(self.braking_defaults))
        _number(self.effective_braking.deceleration_mm_s2, "effective_braking.deceleration_mm_s2")
        _number(self.effective_braking.delay_s, "effective_braking.delay_s", positive=False)
        if self.effective_braking != resolved:
            raise ValueError("effective braking values/provenance mismatch")
        if self.encoder.max_speed_mm_s <= self.motion.max_linear_mm_s:
            raise ValueError("encoder plausible speed must exceed position command limit")
        if self.encoder.feedback_max_age_s > self.pid.dt_max_s:
            raise ValueError("encoder age must fit PID sample interval budget")


@dataclass(frozen=True)
class ServoSettings:
    schema_version: int
    config_revision: str
    backend: str
    device: NativeDynamixelConfig
    manual_axis_to_servo_sign: int
    max_acceleration_rpm_s: float
    control_period_s: float
    io_period_s: float
    feedback_max_age_s: float
    output_ttl_s: float
    stop_timeout_s: float
    settle_s: float
    velocity_tolerance_rpm: float
    lease_max_s: float
    manual_input_max_age_s: float
    queue_capacity: int
    history_capacity: int
    mechanical_mapping: None
    position_pid: None
    sha256: str
    manual_input_mode: str = "qualified_state"
    control_mode: str = "manual"
    position: PositionSettings | None = None

    def __post_init__(self):
        _integer(self.schema_version, "schema_version", 1, 3)
        _text(self.config_revision, "config_revision")
        if type(self.backend) is not str or self.backend not in ("simulation", "hardware"):
            raise ValueError("backend must be simulation or hardware")
        if self.manual_input_mode not in ("qualified_state", "joydev_hold"):
            raise ValueError("unknown native servo manual input mode")
        if self.schema_version == 1 and self.manual_input_mode != "qualified_state":
            raise ValueError("schema 1 retains qualified-state input semantics")
        if self.control_mode not in ("manual", "position"):
            raise ValueError("control_mode must be manual or position")
        if self.schema_version < 3 and (self.control_mode != "manual" or self.position is not None):
            raise ValueError("legacy servo snapshots are manual only")
        if self.control_mode == "position":
            if type(self.position) is not PositionSettings:
                raise ValueError("position mode requires complete PositionSettings")
        elif self.position is not None:
            raise ValueError("manual mode must not contain external position settings")
        if type(self.device) is not NativeDynamixelConfig:
            raise ValueError("device must be NativeDynamixelConfig")
        if self.device.model_number != 1020:
            raise ValueError("native servo path supports only XM430-W350 model 1020")
        if self.device.profile_acceleration_raw is None:
            raise ValueError("native servo path requires explicit finite profile acceleration")
        _integer(self.manual_axis_to_servo_sign, "manual_axis_to_servo_sign", -1, -1)
        for name in (
            "max_acceleration_rpm_s",
            "control_period_s",
            "io_period_s",
            "feedback_max_age_s",
            "output_ttl_s",
            "stop_timeout_s",
            "settle_s",
            "velocity_tolerance_rpm",
            "lease_max_s",
            "manual_input_max_age_s",
        ):
            _number(getattr(self, name), name)
        _integer(self.queue_capacity, "queue_capacity", 1, 4096)
        _integer(self.history_capacity, "history_capacity", 1, 16384)
        if self.mechanical_mapping is not None or self.position_pid is not None:
            raise ValueError("legacy position placeholders must be None; use position settings")
        _text(self.sha256, "sha256", 64)
        if len(self.sha256) != 64 or any(c not in "0123456789abcdef" for c in self.sha256):
            raise ValueError("sha256 must be lowercase hexadecimal")
        # Ordinary motion performs a WRITE/ACK and a separate feedback READ.
        # io_period_s bounds the whole cycle, rather than adding a fixed sleep.
        io_cycle_s = max(2 * self.io_timeout_s, self.io_period_s)
        if io_cycle_s + self.control_period_s >= self.output_ttl_s:
            raise ValueError("write/read cycle plus control period must fit output TTL")
        if (
            self.control_mode == "manual"
            and io_cycle_s + self.control_period_s >= self.manual_input_max_age_s
        ):
            raise ValueError("write/read cycle plus control period must fit manual input age")
        if max(self.control_period_s, self.io_period_s) >= self.output_ttl_s:
            raise ValueError("control and I/O periods must be shorter than output TTL")
        if self.feedback_max_age_s <= io_cycle_s:
            raise ValueError("feedback maximum age must accommodate one bounded I/O cycle")
        if self.stop_timeout_s <= 2 * self.io_timeout_s + self.settle_s:
            raise ValueError("stop timeout must cover independent zero/disable and settling")
        if self.device.watchdog_raw * 0.020 <= self.io_timeout_s + self.io_period_s:
            raise ValueError("watchdog must exceed the bounded ordinary communication interval")
        if self.lease_max_s <= self.output_ttl_s:
            raise ValueError("lease maximum must exceed output TTL")
        if self.velocity_tolerance_rpm >= self.max_rpm:
            raise ValueError("stop tolerance must be below maximum speed")
        if self.position is not None:
            if (
                abs(
                    self.position.mechanics.mapping.velocity_to_rpm(
                        self.position.motion.max_linear_mm_s
                    )
                )
                > self.max_rpm
            ):
                raise ValueError("position speed exceeds native servo maximum")
            if self.control_period_s >= self.position.encoder.feedback_max_age_s:
                raise ValueError("control period must fit encoder feedback age")

    @property
    def control_policy(self):
        return (
            POSITION_EXTERNAL_ENCODER if self.control_mode == "position" else MANUAL_SERVO_VELOCITY
        )

    @property
    def manual_hold_enabled(self):
        return self.manual_input_mode == "joydev_hold"

    @property
    def max_rpm(self):
        return self.device.max_rpm

    @property
    def io_timeout_s(self):
        return self.device.transaction_timeout_s

    @property
    def revision(self):
        return self.config_revision


def _canonical(document):
    return json.dumps(
        document, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")


def _exact(document, expected, name):
    if type(document) is not dict or set(document) != set(expected):
        actual = set(document) if type(document) is dict else set()
        raise ValueError(
            f"{name}: missing={sorted(set(expected) - actual)} "
            f"unknown={sorted(actual - set(expected))}"
        )


def servo_settings_from_document(document) -> ServoSettings:
    """Validate the complete snapshot, including its exact content digest."""
    names = {field.name for field in fields(ServoSettings)}
    if type(document) is dict and document.get("schema_version") in (1, 2):
        names -= {"control_mode", "position"}
    if type(document) is dict and document.get("schema_version") == 1:
        names -= {"manual_input_mode"}
    _exact(document, names, "servo snapshot")
    payload = {key: value for key, value in document.items() if key != "sha256"}
    digest = hashlib.sha256(_canonical(payload)).hexdigest()
    if document["sha256"] != digest:
        raise ValueError("servo configuration digest mismatch; export the source settings again")
    _exact(payload["device"], {field.name for field in fields(NativeDynamixelConfig)}, "device")
    values = dict(document)
    if document["schema_version"] in (1, 2):
        values.update(control_mode="manual", position=None)
    if document["schema_version"] == 1:
        # Version 1 had only qualified-state hardware input. Its original
        # digest is checked above before assigning these explicit semantics.
        values.update(manual_input_mode="qualified_state")
    values["device"] = NativeDynamixelConfig(**values["device"])
    if values["position"] is not None:
        values["position"] = _position_from_document(values["position"])
    return ServoSettings(**values)


def _position_from_document(document):
    _exact(document, {field.name for field in fields(PositionSettings)}, "position")
    values = {}
    for name, cls in (
        ("mechanics", PositionMechanicalSettings),
        ("pid", PositionPIDSettings),
        ("motion", PositionMotionSettings),
        ("braking_defaults", PositionBrakingDefaults),
        ("effective_braking", ResolvedBraking),
        ("encoder", PositionEncoderSettings),
    ):
        _exact(document[name], {field.name for field in fields(cls)}, f"position.{name}")
        values[name] = cls(**document[name])
    return PositionSettings(**values)


def make_servo_settings(payload) -> ServoSettings:
    """Build a validated snapshot from the exporter's complete explicit values."""
    names = {field.name for field in fields(ServoSettings)} - {"sha256"}
    if type(payload) is dict and payload.get("schema_version") in (1, 2):
        names -= {"control_mode", "position"}
    if type(payload) is dict and payload.get("schema_version") == 1:
        names -= {"manual_input_mode"}
    _exact(payload, names, "settings")
    return servo_settings_from_document(
        {**payload, "sha256": hashlib.sha256(_canonical(payload)).hexdigest()}
    )


def servo_settings_document(settings: ServoSettings):
    document = asdict(settings)
    if settings.schema_version in (1, 2):
        del document["control_mode"]
        del document["position"]
    if settings.schema_version == 1:
        del document["manual_input_mode"]
    return document


def make_position_settings(
    runtime_payload, servo_position_draft, encoder_draft, *, config_revision=None
):
    """Combine the two explicit source drafts into a strict schema-3 snapshot.

    Runtime consumers only read the resulting JSON; this helper executes no
    source files and never opens devices. Source metadata not used at runtime
    (board wiring and purchase description) remains in the source scripts.
    """
    if (
        servo_position_draft["kind"] != "servo_position_draft"
        or encoder_draft["kind"] != "encoder_position_draft"
    ):
        raise ValueError("position and encoder source drafts required")
    comm, fmt = encoder_draft["communication"], encoder_draft["count_format"]
    if comm["mode"] != "query" or comm["interface"] != "RS485" or comm["protocol"] != "Modbus RTU":
        raise ValueError("query-mode RS485 Modbus encoder required")
    encoder = {"model": encoder_draft["device"]["model"]}
    for name in (
        "port",
        "baudrate",
        "bytesize",
        "parity",
        "stopbits",
        "address",
        "function_code",
        "register_start",
        "register_count",
        "crc_byte_order",
    ):
        encoder[name] = comm[name]
    for name in (
        "bits",
        "signed",
        "byte_order_in_register",
        "word_order",
        "count_modulus",
        "counts_per_revolution",
    ):
        encoder[name] = fmt[name]
    encoder.update(encoder_draft["mechanics"])
    encoder.update(encoder_draft["acquisition"])
    encoder["velocity_window_s"] = encoder_draft["estimation"]["velocity_window_s"]
    payload = dict(runtime_payload)
    payload.update(
        schema_version=3,
        control_mode="position",
        config_revision=(
            runtime_payload["config_revision"] + "-position-v1"
            if config_revision is None
            else config_revision
        ),
        position={
            **{
                name: servo_position_draft[name]
                for name in ("mechanics", "pid", "motion", "braking_defaults", "effective_braking")
            },
            "encoder": encoder,
        },
    )
    return make_servo_settings(payload)


def _unique_object(pairs):
    document = {}
    for key, value in pairs:
        if key in document:
            raise ValueError(f"duplicate JSON key: {key}")
        document[key] = value
    return document


def load_servo_settings(path) -> ServoSettings:
    with Path(path).open("rb") as stream:
        data = stream.read(65537)
    if len(data) > 65536:
        raise ValueError("servo configuration exceeds 64 KiB")
    return servo_settings_from_document(json.loads(data, object_pairs_hook=_unique_object))
