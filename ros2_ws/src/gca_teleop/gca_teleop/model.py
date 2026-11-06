"""Immutable, ROS-free input contracts; observation is not RF freshness."""

from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from gca.domain.manual import ManualInputQuality


class InputError(RuntimeError):
    def __init__(self, code: str, message: str = ""):
        self.code = code
        super().__init__(message or code)


@dataclass(frozen=True)
class InputIdentity:
    path: str
    name: str
    vendor: int
    product: int
    bus: int
    version: int
    axis_codes: tuple[int, ...]
    button_codes: tuple[int, ...]
    phys: str = ""
    uniq: str = ""
    driver: str = ""
    kernel: str = ""
    device_mode: str = ""

    @property
    def fingerprint(self) -> str:
        # Device numbers may change after hotplug. The stable physical port,
        # observable identity/capabilities/versions and explicit mode still bind.
        value = asdict(self)
        value.pop("path")
        return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()


@dataclass(frozen=True)
class InputFrame:
    source_id: str
    input_epoch: str
    event_seq: int
    raw_event_ns: int
    observation_seq: int
    observed_ns: int
    clock_domain_id: str
    axes: tuple[int | None, ...]
    buttons: tuple[int | None, ...]
    changed_axes: tuple[int, ...] = ()
    changed_buttons: tuple[int, ...] = ()
    initialized: bool = False
    synthetic: bool = False
    resync: bool = False
    identity: InputIdentity | None = None
    kernel_event_ms: int | None = None
    source_kind: str = "JOYDEV"


@dataclass(frozen=True)
class InputHealth:
    connected: bool
    initialized: bool
    reader_heartbeat_ns: int
    source_id: str
    input_epoch: str
    reason: str = ""
    link_evidence: str = "UNKNOWN"
    qualification_id: str = ""
    identity: InputIdentity | None = None


@dataclass(frozen=True)
class MappedInput:
    lx: float
    ly: float
    lt: float
    b: bool
    centered: bool


@dataclass(frozen=True)
class Intent:
    action: str = "NONE"
    linear_mm_s: float = 0.0
    rotation_deg_s: float = 0.0
    deadman: bool = False
    quality: ManualInputQuality | None = None
    reason: str = ""
    gate: str = "WAIT_NEUTRAL"
    lt_pressed: bool = False
    b_pressed: bool = False
    input_epoch: str = ""
