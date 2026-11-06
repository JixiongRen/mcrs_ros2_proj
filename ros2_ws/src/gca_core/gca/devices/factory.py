"""Explicit hardware assembly, with no construction-time I/O or fallback.

Hardware dictionaries are separate from simulation parameters. Every physical
connection, direction and calibration below is required; merely setting one
``hardware_verified`` flag cannot manufacture a working device configuration.
"""

from __future__ import annotations

import time
from dataclasses import fields

from .common import DeviceError, DynamixelConfig, EncoderConfig, PigpioConfig
from .dynamixel import DynamixelServo
from .encoder import SerialEncoder
from .pigpio_engine import PigpioPulseEngine


def _configuration(cls, values, required, timeout_s):
    if not isinstance(values, dict):
        raise DeviceError("CONFIG_INVALID", f"{cls.__name__} must be a mapping")
    unknown = set(values) - {item.name for item in fields(cls)}
    missing = set(required) - set(values)
    if unknown or missing or any(values.get(key) is None for key in required):
        raise DeviceError(
            "CONFIG_INVALID",
            f"{cls.__name__}: missing={sorted(missing)} unknown={sorted(unknown)} or null physical field",
        )
    if values.get("verified") is not True:
        raise DeviceError("HARDWARE_UNVERIFIED", f"{cls.__name__} evidence is not verified")
    candidate = dict(values)
    timeout_name = "socket_timeout_s" if cls is PigpioConfig else "transaction_timeout_s"
    candidate.setdefault(timeout_name, timeout_s)
    try:
        return cls(**candidate)
    except (TypeError, ValueError) as exc:
        raise DeviceError("CONFIG_INVALID", str(exc)) from exc


def hardware_devices(config, clock=time.monotonic_ns):
    """Return (servo, encoder, pulse) adapters, never connected on construction.

    Raises an explicit error for unsupported/missing hardware configuration;
    callers must not catch it to substitute simulated devices.
    """
    if config.backend != "hardware" or config.hardware_verified is not True:
        raise DeviceError("HARDWARE_UNVERIFIED", "hardware backend requires explicit verification")
    hardware = config.hardware
    if not isinstance(hardware, dict) or set(hardware) != {"servo", "encoder", "pulse"}:
        raise DeviceError("CONFIG_INVALID", "hardware must contain exactly servo, encoder, pulse")
    servo = _configuration(
        DynamixelConfig,
        hardware["servo"],
        (
            "port",
            "baudrate",
            "device_id",
            "model_number",
            "mm_per_revolution",
            "velocity_unit_rpm",
            "max_velocity_raw",
            "watchdog_raw",
            "direction_sign",
        ),
        config.io_timeout_s,
    )
    encoder = _configuration(
        EncoderConfig,
        hardware["encoder"],
        ("port", "baudrate", "address", "register", "bits", "signed", "quiet_s", "byteorder"),
        config.io_timeout_s,
    )
    pulse = _configuration(
        PigpioConfig,
        hardware["pulse"],
        (
            "pulse_gpio",
            "direction_gpio",
            "high_us",
            "low_us",
            "direction_setup_us",
            "direction_hold_us",
            "max_segment_s",
            "max_segment_pulses",
            "positive_direction_level",
            "exclusive_wave_owner",
            "waveform_hil_verified",
        ),
        config.io_timeout_s,
    )
    if not pulse.exclusive_wave_owner or not pulse.waveform_hil_verified:
        raise DeviceError("HARDWARE_UNVERIFIED", "finite-wave timing/counting evidence required")
    if (
        config.pulse_high_us != pulse.high_us
        or config.pulse_low_us != pulse.low_us
        or config.direction_setup_us != pulse.direction_setup_us
        or config.direction_hold_us != pulse.direction_hold_us
    ):
        raise DeviceError("CONFIG_INVALID", "runtime/profile and physical pulse timing must match")
    configured_modulus = encoder.modulus or 0
    if config.encoder_count_modulus != configured_modulus:
        raise DeviceError("CONFIG_INVALID", "encoder protocol and runtime count modulus must match")
    if servo.port == encoder.port:
        raise DeviceError("CONFIG_INVALID", "shared physical bus needs a combined I/O backend")
    if pulse.max_segment_s > config.output_ttl_s or config.pulse_segment_s > pulse.max_segment_s:
        raise DeviceError("CONFIG_INVALID", "pulse segment and local output lease mismatch")
    raw_at_limit = abs(
        60 * config.max_linear_mm_s / (servo.mm_per_revolution * servo.velocity_unit_rpm)
    )
    if raw_at_limit > servo.max_velocity_raw + 0.5:
        raise DeviceError("CONFIG_INVALID", "runtime linear speed exceeds servo physical limit")
    return (
        DynamixelServo(servo, clock=clock),
        SerialEncoder(encoder, clock=clock),
        PigpioPulseEngine(pulse, clock=clock),
    )
