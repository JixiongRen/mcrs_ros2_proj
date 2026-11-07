"""Bind preserved mapping evidence to the observed Asura 3S BFM mode.

This deliberately supports the witnessed BFM/PC360/BFM round trip only. It
does not identify arbitrary controllers or grant wireless qualification. Local
hashes detect changed records; they are not signatures against a malicious
local administrator. Importing or validating this module never opens a device.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from dataclasses import asdict, fields, replace
from datetime import datetime
from pathlib import Path
from uuid import uuid4

from .config import (
    bounded_string,
    bounded_text,
    finite,
    integer,
    known_device_mode,
    load_profile,
    validate_mapping_evidence,
)
from .model import InputIdentity

BFM_MODE = "BEITONG_Asura3S_2.4GHz_BFM"
BFM_NAME = "BEITONG  BEITONG AXIULUO3SNS BFM DONGLE"
PC360_NAME = "Microsoft X-Box 360 pad"
METHOD = "ASURA3S_OPERATOR_WITNESSED_BFM_PC360_BFM"
MANUAL_URL = (
    "https://game.cdn.betophall.com/ckfinder/userfiles/files/"
    "%E8%8B%B1%E6%96%87%E7%94%B5%E5%AD%90%E8%AF%B4%E6%98%8E%E4%B9%A6/"
    "%E8%AF%B4%E6%98%8E%E4%B9%A6/%E7%AB%AF%E6%B8%B8%E6%89%8B%E6%9F%84/"
    "user%20manual-%20Asura%203S%20game%20controller%20Multi-mode%20-"
    "%20BTP-2650NS%20-%20V2_0.pdf"
)
DOCUMENTS = ("mapping.json", "assessment.json", "observations.jsonl", "operator_actions.json")
_TOP_KEYS = {
    "schema_version",
    "record_kind",
    "status",
    "source_kind",
    "record_id",
    "identity",
    "identity_fingerprint",
    "mapping_fingerprint",
    "confirmed_at",
    "mode_basis",
    "source_documents",
    "wireless_qualification",
}


def _sha(text):
    return hashlib.sha256(text.encode("utf8")).hexdigest()


def _loads(text):
    def invalid_constant(value):
        raise ValueError(f"Non-finite JSON value: {value}")

    def unique_object(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError("Duplicate JSON field")
            result[key] = value
        return result

    try:
        return json.loads(text, parse_constant=invalid_constant, object_pairs_hook=unique_object)
    except RecursionError:
        raise ValueError("Mode evidence nesting exceeds limit") from None


def _identity(value):
    if not isinstance(value, dict) or set(value) != {f.name for f in fields(InputIdentity)}:
        raise ValueError("MODE_IDENTITY_FIELDS")
    value = dict(value)
    for name in ("vendor", "product", "bus", "version"):
        integer(value[name], name)
    for name in ("axis_codes", "button_codes"):
        codes = value[name]
        if not isinstance(codes, (list, tuple)) or not 1 <= len(codes) <= 256:
            raise ValueError("MODE_IDENTITY_CAPABILITIES")
        for code in codes:
            integer(code, name, 0, 65535)
        if len(set(codes)) != len(codes):
            raise ValueError("MODE_IDENTITY_CAPABILITIES")
        value[name] = tuple(codes)
    for name in ("path", "name", "phys", "uniq", "driver", "kernel", "device_mode"):
        bounded_string(value[name], name, required=name != "uniq")
    return InputIdentity(**value)


def _time(value):
    bounded_string(value, "mode record UTC")
    try:
        result = datetime.fromisoformat(value)
    except ValueError:
        raise ValueError("MODE_RECORD_TIME") from None
    if result.tzinfo is None:
        raise ValueError("MODE_RECORD_TIME")
    return result


def _documents(data):
    documents = data["source_documents"]
    if not isinstance(documents, dict) or set(documents) != set(DOCUMENTS):
        raise ValueError("MODE_SOURCE_DOCUMENTS")
    texts, total = {}, 0
    for name in DOCUMENTS:
        item = documents[name]
        if not isinstance(item, dict) or set(item) != {"text", "sha256"}:
            raise ValueError("MODE_SOURCE_DOCUMENTS")
        text = item["text"]
        if not isinstance(text, str):
            raise ValueError("MODE_SOURCE_DOCUMENTS")
        total += len(text.encode("utf8"))
        if total > 1_900_000:
            raise ValueError("Mode evidence exceeds loader capacity")
        if item["sha256"] != _sha(text):
            raise ValueError("MODE_SOURCE_HASH_MISMATCH")
        texts[name] = text
    return texts


def _bitmap(value):
    # The recorded Pi sysfs capture uses 64-bit unsigned-long words, most
    # significant first. This format is part of this narrowly scoped method.
    if not isinstance(value, str) or not 1 <= len(value) <= 256:
        raise ValueError("MODE_CAPABILITY_BITMAP")
    result = 0
    for word in value.split():
        number = int(word, 16)
        if not 0 <= number < 2**64:
            raise ValueError("MODE_CAPABILITY_BITMAP")
        result = (result << 64) | number
    return result


def _roundtrip(text, identity):
    lines = text.splitlines()
    if not 3 <= len(lines) <= 10_000:
        raise ValueError("MODE_OBSERVATION_CAPACITY")
    modes, last_elapsed, last_utc = [], -1.0, None
    for line in lines:
        row = _loads(line)
        if not isinstance(row, dict):
            raise ValueError("MODE_OBSERVATION_RECORD")
        elapsed = row.get("elapsed_s")
        finite(elapsed, "mode elapsed seconds", 0, 3600)
        stamp = _time(row.get("utc"))
        if elapsed < last_elapsed or last_utc is not None and stamp < last_utc:
            raise ValueError("MODE_OBSERVATION_ORDER")
        last_elapsed, last_utc = elapsed, stamp
        if row.get("kind") != "metadata":
            continue
        devices = row.get("devices")
        if not isinstance(devices, list) or len(devices) > 1:
            raise ValueError("MODE_DEVICE_AMBIGUOUS")
        if not devices:  # Preserve hot-unplug records without inventing a mode.
            continue
        device = devices[0]
        if not isinstance(device, dict):
            raise ValueError("MODE_DEVICE_FIELDS")
        observed, drivers = device.get("fields"), device.get("drivers")
        if not isinstance(observed, dict) or not isinstance(drivers, list):
            raise ValueError("MODE_DEVICE_FIELDS")
        if not drivers or not all(isinstance(d, str) for d in drivers):
            raise ValueError("MODE_DEVICE_DRIVER")
        if observed.get("phys") != identity.phys or observed.get("uniq") != identity.uniq:
            raise ValueError("MODE_PHYSICAL_PORT_MISMATCH")
        signature = (observed.get("name"), observed.get("id/vendor"), observed.get("id/product"))
        if signature == (BFM_NAME, "20bc", "506d"):
            mode, expected_driver = "BFM", "/sys/bus/hid/drivers/hid-generic"
            if (
                int(observed["id/bustype"], 16) != identity.bus
                or int(observed["id/version"], 16) != identity.version
                or _bitmap(observed["capabilities/abs"]) != sum(1 << c for c in identity.axis_codes)
                or _bitmap(observed["capabilities/key"])
                != sum(1 << c for c in identity.button_codes)
            ):
                raise ValueError("MODE_CAPABILITIES_MISMATCH")
        elif signature == (PC360_NAME, "045e", "028e"):
            mode, expected_driver = "PC360", "/sys/bus/usb/drivers/xpad"
            if int(observed["id/bustype"], 16) != 3:
                raise ValueError("MODE_DEVICE_FIELDS")
        else:
            raise ValueError("MODE_UNEXPECTED_USB_IDENTITY")
        if expected_driver not in drivers:
            # The actual capture includes a driver-unbound snapshot immediately
            # before each disappearance. Any unrelated input driver is invalid.
            if any("/sys/bus/hid/drivers/" in d or d.endswith("/xpad") for d in drivers):
                raise ValueError("MODE_DEVICE_DRIVER")
            continue
        if not modes or modes[-1] != mode:
            modes.append(mode)
    if modes != ["BFM", "PC360", "BFM"]:
        raise ValueError("MODE_ROUNDTRIP_REQUIRED")
    return last_utc


def validate_mode_confirmation(profile, identity, data):
    """Validate a source-preserving association, never wireless qualification."""
    if not isinstance(data, dict) or set(data) != _TOP_KEYS:
        raise ValueError("MODE_CONFIRMATION_FIELDS")
    if (
        type(data["schema_version"]) is not int
        or data["schema_version"] != 3
        or data["record_kind"] != "MODE_CONFIRMATION"
        or data["status"] != "PASS"
        or data["source_kind"] != "JOYDEV_READONLY"
        or data["wireless_qualification"] != "NOT_RUN"
        or profile.source != "joydev"
        or profile.device.device_mode != BFM_MODE
        or identity.device_mode != BFM_MODE
        or data["identity_fingerprint"] != identity.fingerprint
        or data["mapping_fingerprint"] != profile.mapping.fingerprint
        or data["mode_basis"] != {"method": METHOD, "manufacturer_manual_url": MANUAL_URL}
    ):
        raise ValueError("MODE_CONFIRMATION_MISMATCH")
    current = _identity(data["identity"])
    if current.fingerprint != identity.fingerprint:
        raise ValueError("MODE_IDENTITY_MISMATCH")
    if (
        current.name != BFM_NAME
        or current.vendor != 0x20BC
        or current.product != 0x506D
        or current.bus != 3
        or current.driver != "hid-generic"
    ):
        raise ValueError("MODE_UNSUPPORTED_DEVICE")
    texts = _documents(data)
    original = _loads(texts["mapping.json"])
    if (
        not isinstance(original, dict)
        or type(original.get("schema_version")) is not int
        or original["schema_version"] != 2
    ):
        raise ValueError("MODE_ORIGINAL_SCHEMA2_REQUIRED")
    old = _identity(original.get("identity"))
    if known_device_mode(old.device_mode):
        raise ValueError("MODE_ORIGINAL_MUST_BE_UNKNOWN")
    if replace(old, path=current.path, device_mode=BFM_MODE) != current:
        raise ValueError("MODE_IDENTITY_CHANGED")
    old_profile = replace(profile, device=replace(profile.device, device_mode=old.device_mode))
    validate_mapping_evidence(old_profile, old, evidence=original)
    assessment = _loads(texts["assessment.json"])
    actions = _loads(texts["operator_actions.json"])
    if not isinstance(assessment, dict) or not isinstance(actions, dict):
        raise ValueError("MODE_CONFIRMATION_RECORDS")
    for name, document in (
        ("historical_mapping_sha256", "mapping.json"),
        ("observations_sha256", "observations.jsonl"),
        ("operator_actions_sha256", "operator_actions.json"),
    ):
        if assessment.get(name) != _sha(texts[document]):
            raise ValueError("MODE_ASSESSMENT_HASH_MISMATCH")
    if (
        type(assessment.get("schema_version")) is not int
        or assessment["schema_version"] != 1
        or assessment.get("mode_confirmation_status") != "PASS"
        or assessment.get("initial_mode") != "BFM"
        or assessment.get("intermediate_mode") != "PC360 (Xbox 360 compatible)"
        or assessment.get("final_mode") != "BFM"
        or assessment.get("operator_completion_received") is not True
        or assessment.get("actuator_accessed") is not False
        or assessment.get("wireless_qualification") != "NOT_RUN"
        or assessment.get("current_identity_fingerprint") != current.fingerprint
        or _identity(assessment.get("current_identity")).fingerprint != current.fingerprint
    ):
        raise ValueError("MODE_ASSESSMENT_MISMATCH")
    replies = actions.get("actions")
    if (
        actions.get("source") != "operator_replies"
        or not isinstance(replies, list)
        or len(replies) != 2
    ):
        raise ValueError("MODE_OPERATOR_REPLIES_REQUIRED")
    for item, completed in zip(replies, ("已长按", "第二次长按完成"), strict=True):
        if not isinstance(item, dict) or set(item) != {"instruction", "reply"}:
            raise ValueError("MODE_OPERATOR_REPLIES_REQUIRED")
        bounded_string(item["instruction"], "operator instruction")
        bounded_string(item["reply"], "operator reply")
        if not item["reply"].startswith(completed):
            raise ValueError("MODE_OPERATOR_COMPLETION_REQUIRED")
    observed_until = _roundtrip(texts["observations.jsonl"], current)
    if (
        data["confirmed_at"] != assessment.get("recorded_utc")
        or _time(data["confirmed_at"]) < observed_until
        or _time(actions.get("recorded_utc")) < observed_until
    ):
        raise ValueError("MODE_CONFIRMATION_TIME")
    bounded_string(data["record_id"], "mode mapping ID")
    return data["record_id"]


def build_mode_confirmation(profile, identity, source_texts):
    """Build from exact source text; callers retain the original files."""
    if set(source_texts) != set(DOCUMENTS):
        raise ValueError("MODE_SOURCE_DOCUMENTS")
    assessment = _loads(source_texts["assessment.json"])
    result = {
        "schema_version": 3,
        "record_kind": "MODE_CONFIRMATION",
        "status": "PASS",
        "source_kind": "JOYDEV_READONLY",
        "record_id": uuid4().hex,
        "identity": asdict(identity),
        "identity_fingerprint": identity.fingerprint,
        "mapping_fingerprint": profile.mapping.fingerprint,
        "confirmed_at": assessment["recorded_utc"],
        "mode_basis": {"method": METHOD, "manufacturer_manual_url": MANUAL_URL},
        "source_documents": {
            name: {"text": text, "sha256": _sha(text)} for name, text in source_texts.items()
        },
        "wireless_qualification": "NOT_RUN",
    }
    validate_mode_confirmation(profile, identity, result)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--observation-directory", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--profile-output", required=True)
    args = parser.parse_args(argv)
    old = load_profile(args.profile)
    profile = replace(
        old,
        profile_id="beitong-20bc-506d-confirmed-bfm-event-age",
        device=replace(old.device, device_mode=BFM_MODE),
        mode="event_age",
        qualification_file="",
    )
    directory = Path(args.observation_directory)
    texts = {"mapping.json": bounded_text(old.mapping_evidence, 1_900_000)}
    for name in DOCUMENTS[1:]:
        texts[name] = bounded_text(directory / name, 1_900_000)
    from .input.joydev import discover
    from .probe import _write, _write_profile

    # Explicit CLI execution alone performs read-only capability discovery.
    identity = discover(profile.device)
    result = build_mode_confirmation(profile, identity, texts)
    output, profile_output = Path(args.output).resolve(), Path(args.profile_output).resolve()
    if output == profile_output or output.exists() or profile_output.exists():
        raise FileExistsError("Mode confirmation outputs must be distinct new files")
    _write(output, result)
    _write_profile(profile_output, replace(profile, mapping_evidence=str(output)))
    print(
        json.dumps(
            {"mapping": str(output), "profile": str(profile_output), "qualification": "NOT_RUN"}
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
