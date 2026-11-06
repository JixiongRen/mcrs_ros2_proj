"""Shared finite-edge contract, independent from ROS and GPIO libraries."""

from math import isfinite

from gca.domain.models import PulseSegment


def validate_edge_offsets(
    segment: PulseSegment, *, high_us: int, low_us: int, setup_us: int, hold_us: int
) -> tuple[int, ...] | None:
    """Validate rising-edge offsets; None explicitly selects legacy uniform mode.

    The exact finite segment includes direction guards and each pulse's full
    high/low time. A supplied profile is never silently converted to uniform.
    """
    edges = segment.edge_offsets_us
    if edges is None:
        return None
    if (
        type(edges) is not tuple
        or len(edges) != abs(segment.signed_pulses)
        or not edges
        or len(edges) > 100_000
        or not isfinite(segment.duration_s)
    ):
        raise ValueError("edge profile/count mismatch or mutable profile")
    duration_us = round(segment.duration_s * 1e6)
    if any(type(t) is not int or t < 0 for t in edges):
        raise ValueError("edge offsets must be nonnegative integer microseconds")
    if edges[0] < setup_us or edges[-1] + high_us + max(low_us, hold_us) > duration_us:
        raise ValueError("edge profile violates direction/terminal timing")
    if any(b - a < high_us + low_us for a, b in zip(edges, edges[1:])):
        raise ValueError("edge profile violates minimum high/low period")
    return edges
