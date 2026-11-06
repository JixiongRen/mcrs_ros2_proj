"""Pure independent-axis normalization; right-side controls never drive GCA."""

from __future__ import annotations

from math import copysign, isfinite

from .model import InputError, MappedInput


def stick(value, binding, deadzone, exponent):
    if type(value) is not int or not binding.minimum <= value <= binding.maximum:
        raise InputError("INVALID_AXIS_RANGE")
    span = (
        binding.maximum - binding.center
        if value >= binding.center
        else binding.center - binding.minimum
    )
    normalized = binding.sign * (value - binding.center) / span
    if abs(normalized) <= deadzone:
        return 0.0
    return copysign(((abs(normalized) - deadzone) / (1 - deadzone)) ** exponent, normalized)


def map_input(frame, config):
    mapping = config.mapping
    if not mapping.complete:
        raise InputError("MAPPING_UNVERIFIED")
    try:
        lx = stick(
            frame.axes[mapping.lx.index], mapping.lx, config.deadzone, config.response_exponent
        )
        ly = stick(
            frame.axes[mapping.ly.index], mapping.ly, config.deadzone, config.response_exponent
        )
        raw = frame.axes[mapping.lt.index]
        if type(raw) is not int:
            raise InputError("INVALID_TRIGGER")
        lt = (raw - mapping.lt.released) / (mapping.lt.pressed - mapping.lt.released)
        if not isfinite(lt) or not -0.02 <= lt <= 1.02:
            raise InputError("INVALID_TRIGGER_RANGE")
        b = frame.buttons[mapping.b_button]
        if type(b) is not int or b not in (0, 1):
            raise InputError("INVALID_BUTTON")
        return MappedInput(lx, ly, min(1.0, max(0.0, lt)), bool(b), lx == 0 and ly == 0)
    except IndexError as exc:
        raise InputError("CAPABILITY_MISMATCH") from exc
