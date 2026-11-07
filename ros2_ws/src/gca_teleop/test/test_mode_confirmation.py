"""Offline provenance tests; fixture records never qualify physical hardware."""

import copy
import hashlib
import json
from dataclasses import asdict, replace

import pytest
import yaml
from gca_teleop.config import DeviceSelector, load_profile, validate_mapping_evidence
from gca_teleop.mode_confirmation import (
    BFM_MODE,
    BFM_NAME,
    PC360_NAME,
    build_mode_confirmation,
    main,
)
from gca_teleop.state_machine import TeleopPipeline
from test_input_pipeline import physical_evidence


def digest(text):
    return hashlib.sha256(text.encode()).hexdigest()


def bitmap(codes):
    value = sum(1 << c for c in codes)
    words = []
    while value:
        words.append(f"{value & (2**64 - 1):x}")
        value >>= 64
    return " ".join(reversed(words)) or "0"


@pytest.fixture
def binding(tmp_path):
    profile, original_identity, source = physical_evidence(tmp_path)
    old = replace(
        original_identity,
        name=BFM_NAME,
        version=273,
        axis_codes=(0, 1, 2, 5, 9, 10, 16, 17),
        button_codes=tuple(range(304, 319)) + (398,),
        driver="hid-generic",
        device_mode="operator_reported_unknown_2.4GHz",
    )
    current = replace(old, device_mode=BFM_MODE)
    original = json.loads(source.read_text())
    original.update(identity=asdict(old), identity_fingerprint=old.fingerprint)
    original_text = json.dumps(original)
    source.write_text(original_text)
    profile = replace(
        profile,
        device=DeviceSelector(
            vendor=current.vendor,
            product=current.product,
            name=current.name,
            axis_codes=current.axis_codes,
            button_codes=current.button_codes,
            device_mode=BFM_MODE,
        ),
    )
    fields = {
        "name": BFM_NAME,
        "id/vendor": "20bc",
        "id/product": "506d",
        "id/bustype": "0003",
        "id/version": "0111",
        "phys": current.phys,
        "uniq": current.uniq,
        "capabilities/abs": bitmap(current.axis_codes),
        "capabilities/key": bitmap(current.button_codes),
    }
    bfm = {
        "path": "/dev/input/js0",
        "sysfs": "/sys/fixture",
        "fields": fields,
        "drivers": ["/sys/bus/hid/drivers/hid-generic", "/sys/bus/usb/drivers/usbhid"],
    }
    pc360 = copy.deepcopy(bfm)
    pc360["fields"].update({"name": PC360_NAME, "id/vendor": "045e", "id/product": "028e"})
    pc360["drivers"] = ["/sys/bus/usb/drivers/xpad"]
    unbound = copy.deepcopy(bfm)
    unbound["drivers"] = ["/sys/bus/usb/drivers/usb"]
    rows = [
        {
            "utc": f"2026-11-07T01:44:0{i}+00:00",
            "elapsed_s": float(i),
            "kind": "metadata",
            "devices": devices,
        }
        for i, devices in enumerate(([bfm], [unbound], [], [pc360], [], [bfm]))
    ]
    actions = {
        "source": "operator_replies",
        "recorded_utc": "2026-11-07T01:44:06+00:00",
        "actions": [
            {"instruction": "Hold LOGO, release and wait", "reply": "已长按 . 指示灯仍常亮"},
            {"instruction": "Repeat LOGO hold to restore BFM", "reply": "第二次长按完成"},
        ],
    }
    texts = {
        "mapping.json": original_text,
        "observations.jsonl": "".join(json.dumps(r) + "\n" for r in rows),
        "operator_actions.json": json.dumps(actions),
    }
    assessment = {
        "schema_version": 1,
        "mode_confirmation_status": "PASS",
        "recorded_utc": "2026-11-07T01:44:07+00:00",
        "initial_mode": "BFM",
        "intermediate_mode": "PC360 (Xbox 360 compatible)",
        "final_mode": "BFM",
        "operator_completion_received": True,
        "actuator_accessed": False,
        "wireless_qualification": "NOT_RUN",
        "current_identity": asdict(current),
        "current_identity_fingerprint": current.fingerprint,
        "historical_mapping_sha256": digest(texts["mapping.json"]),
        "observations_sha256": digest(texts["observations.jsonl"]),
        "operator_actions_sha256": digest(texts["operator_actions.json"]),
    }
    texts["assessment.json"] = json.dumps(assessment)
    data = build_mode_confirmation(profile, current, texts)
    return profile, current, data, source, texts


def replace_document(data, name, text):
    """Rehash mutations so rejection must come from content validation."""
    data["source_documents"][name] = {"text": text, "sha256": digest(text)}
    assessment = json.loads(data["source_documents"]["assessment.json"]["text"])
    key = {
        "mapping.json": "historical_mapping_sha256",
        "observations.jsonl": "observations_sha256",
        "operator_actions.json": "operator_actions_sha256",
    }[name]
    assessment[key] = digest(text)
    updated = json.dumps(assessment)
    data["source_documents"]["assessment.json"] = {"text": updated, "sha256": digest(updated)}


def test_confirmed_mapping_loads_pipeline_but_does_not_qualify_hardware(binding, tmp_path):
    profile, identity, data, _, _ = binding
    path = tmp_path / "confirmed.json"
    path.write_text(json.dumps(data))
    profile = replace(profile, mapping_evidence=str(path))
    assert validate_mapping_evidence(profile, identity) == data["record_id"]
    pipeline = TeleopPipeline(profile)
    assert not pipeline._record_error
    assert pipeline._record_identity_fingerprint == identity.fingerprint
    assert not pipeline._verified_qualification
    assert pipeline.hardware_arm_error(None, 0) == "HARDWARE_INPUT_NOT_QUALIFIED"


def test_cli_preserves_originals_and_writes_only_inactive_profile(binding, tmp_path, monkeypatch):
    profile, identity, _, source, texts = binding
    old = replace(
        profile, device=replace(profile.device, device_mode="operator_reported_unknown_2.4GHz")
    )
    old_profile = tmp_path / "old.yaml"
    old_profile.write_text(yaml.safe_dump(asdict(old)))
    capture = tmp_path / "capture"
    capture.mkdir()
    for name in ("assessment.json", "observations.jsonl", "operator_actions.json"):
        (capture / name).write_text(texts[name])
    preserved = {p: p.read_bytes() for p in (source, old_profile, *capture.iterdir())}
    monkeypatch.setattr("gca_teleop.input.joydev.discover", lambda selector: identity)
    output, output_profile = tmp_path / "confirmed.json", tmp_path / "confirmed.yaml"
    assert (
        main(
            [
                "--profile",
                str(old_profile),
                "--observation-directory",
                str(capture),
                "--output",
                str(output),
                "--profile-output",
                str(output_profile),
            ]
        )
        == 0
    )
    generated = load_profile(output_profile)
    assert generated.mode == "event_age" and not generated.qualification_file
    assert generated.device.device_mode == BFM_MODE
    assert validate_mapping_evidence(generated, identity)
    assert all(p.read_bytes() == content for p, content in preserved.items())


def test_source_byte_mutation_without_rehash_is_rejected(binding):
    profile, identity, data, _, _ = binding
    data["source_documents"]["mapping.json"]["text"] += " "
    with pytest.raises(ValueError, match="SOURCE_HASH"):
        validate_mapping_evidence(profile, identity, evidence=data)


def test_other_mode_and_changed_identity_are_not_relabelled(binding):
    profile, identity, data, _, _ = binding
    with pytest.raises(ValueError, match="MODE_CONFIRMATION_MISMATCH"):
        validate_mapping_evidence(
            replace(profile, device=replace(profile.device, device_mode="PC360")),
            identity,
            evidence=data,
        )
    changed = replace(identity, axis_codes=(*identity.axis_codes[:-1], 18))
    data.update(identity=asdict(changed), identity_fingerprint=changed.fingerprint)
    with pytest.raises(ValueError, match="MODE_IDENTITY_CHANGED"):
        validate_mapping_evidence(profile, changed, evidence=data)


def test_nested_wrapper_cannot_replace_original_schema2(binding):
    profile, identity, data, _, _ = binding
    source = json.loads(data["source_documents"]["mapping.json"]["text"])
    source["schema_version"] = 3
    replace_document(data, "mapping.json", json.dumps(source))
    with pytest.raises(ValueError, match="ORIGINAL_SCHEMA2"):
        validate_mapping_evidence(profile, identity, evidence=data)


def test_pass_flags_cannot_replace_roundtrip_or_capability_match(binding):
    profile, identity, data, _, _ = binding
    rows = [
        json.loads(s) for s in data["source_documents"]["observations.jsonl"]["text"].splitlines()
    ]
    missing_middle = copy.deepcopy(data)
    rows_without_pc = [r for i, r in enumerate(rows) if i != 3]
    replace_document(
        missing_middle, "observations.jsonl", "".join(json.dumps(r) + "\n" for r in rows_without_pc)
    )
    with pytest.raises(ValueError, match="ROUNDTRIP_REQUIRED"):
        validate_mapping_evidence(profile, identity, evidence=missing_middle)
    rows[-1]["devices"][0]["fields"]["capabilities/abs"] = "0"
    replace_document(data, "observations.jsonl", "".join(json.dumps(r) + "\n" for r in rows))
    with pytest.raises(ValueError, match="CAPABILITIES_MISMATCH"):
        validate_mapping_evidence(profile, identity, evidence=data)


@pytest.mark.parametrize("corruption", ["time_reversal", "nonfinite", "ambiguous"])
def test_observation_time_and_unique_device_are_required(binding, corruption):
    profile, identity, data, _, _ = binding
    rows = [
        json.loads(s) for s in data["source_documents"]["observations.jsonl"]["text"].splitlines()
    ]
    if corruption == "time_reversal":
        rows[-1]["elapsed_s"] = 0
    elif corruption == "nonfinite":
        rows[-1]["elapsed_s"] = float("nan")
    else:
        rows[-1]["devices"] *= 2
    replace_document(data, "observations.jsonl", "".join(json.dumps(r) + "\n" for r in rows))
    with pytest.raises(ValueError):
        validate_mapping_evidence(profile, identity, evidence=data)


def test_unconfirmed_operator_reply_does_not_become_confirmation(binding):
    profile, identity, data, _, _ = binding
    actions = json.loads(data["source_documents"]["operator_actions.json"]["text"])
    actions["actions"][1]["reply"] = "未操作"
    replace_document(data, "operator_actions.json", json.dumps(actions))
    with pytest.raises(ValueError, match="OPERATOR_COMPLETION_REQUIRED"):
        validate_mapping_evidence(profile, identity, evidence=data)
