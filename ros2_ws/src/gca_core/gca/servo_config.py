"""Strict, ROS-independent configuration for the native servo velocity path.

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

    def __post_init__(self):
        _integer(self.schema_version, "schema_version", 1, 2)
        _text(self.config_revision, "config_revision")
        if type(self.backend) is not str or self.backend not in ("simulation", "hardware"):
            raise ValueError("backend must be simulation or hardware")
        if self.manual_input_mode not in ("qualified_state", "joydev_hold"):
            raise ValueError("unknown native servo manual input mode")
        if self.schema_version == 1 and self.manual_input_mode != "qualified_state":
            raise ValueError("schema 1 retains qualified-state input semantics")
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
            raise ValueError("external position control is unavailable in native servo settings")
        _text(self.sha256, "sha256", 64)
        if len(self.sha256) != 64 or any(c not in "0123456789abcdef" for c in self.sha256):
            raise ValueError("sha256 must be lowercase hexadecimal")
        # Ordinary motion performs a WRITE/ACK and a separate feedback READ.
        # io_period_s bounds the whole cycle, rather than adding a fixed sleep.
        io_cycle_s = max(2 * self.io_timeout_s, self.io_period_s)
        if io_cycle_s + self.control_period_s >= self.output_ttl_s:
            raise ValueError("write/read cycle plus control period must fit output TTL")
        if io_cycle_s + self.control_period_s >= self.manual_input_max_age_s:
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
    if type(document) is dict and document.get("schema_version") == 1:
        names -= {"manual_input_mode"}
    _exact(document, names, "servo snapshot")
    payload = {key: value for key, value in document.items() if key != "sha256"}
    digest = hashlib.sha256(_canonical(payload)).hexdigest()
    if document["sha256"] != digest:
        raise ValueError("servo configuration digest mismatch; export the source settings again")
    _exact(payload["device"], {field.name for field in fields(NativeDynamixelConfig)}, "device")
    values = dict(document)
    if document["schema_version"] == 1:
        # Version 1 had only qualified-state hardware input. Its original
        # digest is checked above before assigning these explicit semantics.
        values.update(manual_input_mode="qualified_state")
    values["device"] = NativeDynamixelConfig(**values["device"])
    return ServoSettings(**values)


def make_servo_settings(payload) -> ServoSettings:
    """Build a validated snapshot from the exporter's complete explicit values."""
    names = {field.name for field in fields(ServoSettings)} - {"sha256"}
    if type(payload) is dict and payload.get("schema_version") == 1:
        names -= {"manual_input_mode"}
    _exact(payload, names, "settings")
    return servo_settings_from_document(
        {**payload, "sha256": hashlib.sha256(_canonical(payload)).hexdigest()}
    )


def servo_settings_document(settings: ServoSettings):
    document = asdict(settings)
    if settings.schema_version == 1:
        del document["manual_input_mode"]
    return document


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
