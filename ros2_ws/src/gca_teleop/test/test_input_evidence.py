"""Offline mapping and qualification replay regressions; never open input devices."""

import copy
import hashlib
import json
from dataclasses import replace

import pytest
from gca_teleop.config import (
    load_profile,
    read_evidence,
    validate_mapping_evidence,
    validate_qualification,
)
from gca_teleop.input.scripted import load_script
from gca_teleop.model import InputFrame
from gca_teleop.probe import MappingWizard, QualificationWizard
from test_input_pipeline import Frames, physical_evidence, qualified_profile, synthetic


def wizard_from_file(profile, identity):
    data = json.loads(open(profile.mapping_evidence).read())
    wizard = MappingWizard(identity)
    for label, records in data["raw_records"].items():
        frames = []
        for record in records:
            record = dict(record)
            for key in ("axes", "buttons", "changed_axes", "changed_buttons"):
                record[key] = tuple(record[key])
            frames.append(InputFrame(**record, identity=identity))
        wizard.capture(label, frames)
    return wizard


def with_axes(frame, updates, *, seq_increment=1):
    axes = list(frame.axes)
    for index, value in updates.items():
        axes[index] = value
    return replace(
        frame,
        axes=tuple(axes),
        changed_axes=tuple(updates),
        event_seq=frame.event_seq + seq_increment,
        observed_ns=frame.observed_ns + seq_increment * 1_000_000,
        raw_event_ns=frame.raw_event_ns + seq_increment * 1_000_000,
    )


def test_jt02_mapping_uses_witnessed_peak_when_operator_recenters_before_reply(tmp_path):
    profile, identity, _ = physical_evidence(tmp_path)
    wizard = wizard_from_file(profile, identity)
    for label, axis in (("lx_left", 0), ("lx_right", 0), ("ly_up", 1), ("ly_down", 1)):
        frames = wizard.steps[label]
        wizard.capture(label, [*frames, with_axes(frames[-1], {axis: 0})])
    mapping, evidence = wizard.build()
    assert mapping == profile.mapping
    assert evidence["selected_samples"]["lx_right"]["axes"][0] == 32767
    assert wizard.steps["lx_right"][-1].axes[0] == 0


def test_jt02_mapping_latest_equal_peak_handles_transition_from_opposite_old_endpoint(tmp_path):
    profile, identity, _ = physical_evidence(tmp_path)
    wizard = wizard_from_file(profile, identity)
    base = wizard.steps["ly_down"][-1]
    old = with_axes(base, {1: -32767})
    newest = with_axes(base, {1: 32767}, seq_increment=2)
    wizard.capture("ly_down", [old, newest])
    assert wizard.build()[0].ly.index == 1


def test_jt02_mapping_accepts_identifiable_natural_diagonal_and_rejects_tie(tmp_path):
    profile, identity, _ = physical_evidence(tmp_path)
    wizard = wizard_from_file(profile, identity)
    for label, cross in (("lx_left", -4392), ("lx_right", 14863)):
        frames = wizard.steps[label]
        wizard.capture(label, [with_axes(frames[-1], {0: frames[-1].axes[0], 1: cross})])
    assert wizard.build()[0].lx.index == 0
    for label, cross in (("lx_left", -32768), ("lx_right", 32767)):
        wizard.capture(
            label,
            [with_axes(wizard.steps[label][-1], {0: wizard.steps[label][-1].axes[0], 1: cross})],
        )
    with pytest.raises(ValueError, match="Ambiguous"):
        wizard.build()


def test_jt02_bad_center_is_not_replaced_by_invented_zero(tmp_path):
    profile, identity, _ = physical_evidence(tmp_path)
    wizard = wizard_from_file(profile, identity)
    wizard.capture("ly_center", [with_axes(wizard.steps["ly_center"][-1], {1: 20268})])
    with pytest.raises(ValueError, match="not neutral"):
        wizard.build()


def test_jt04_pressed_then_released_button_is_still_observed_and_wrong_button_noise_is_excluded(
    tmp_path,
):
    profile, identity, _ = physical_evidence(tmp_path)
    wizard = wizard_from_file(profile, identity)
    original = wizard.steps["b_pressed"][-1]
    buttons = list(original.buttons)
    buttons[1] = 0
    wizard.capture(
        "b_pressed",
        [
            original,
            replace(
                original,
                buttons=tuple(buttons),
                changed_buttons=(1,),
                event_seq=original.event_seq + 1,
            ),
        ],
    )
    assert wizard.build()[0].b_button == 1
    rb = wizard.steps["rb_pressed"][-1]
    noise = list(rb.buttons)
    noise[9] = 1
    wizard.capture("rb_pressed", [replace(rb, buttons=tuple(noise), changed_buttons=(9,)), rb])
    assert wizard.build()[0].reserved_buttons == (5, 9)  # fixture RS is 9


def test_jt04_lt_release_endpoint_refined_only_from_actual_confirmed_axis_events(tmp_path):
    profile, identity, _ = physical_evidence(tmp_path)
    wizard = wizard_from_file(profile, identity)
    release = with_axes(wizard.steps["lt_released"][-1], {2: -29053})
    wizard.capture("lt_released", [release])
    rt = wizard.steps["rt_pressed"][-1]
    wizard.capture("rt_pressed", [with_axes(rt, {2: -32767}), rt])
    mapping, evidence = wizard.build()
    assert mapping.lt.released == -32767
    assert evidence["selected_samples"]["lt_released"]["observed_in"] == "rt_pressed"
    # Canonical JSON sorts labels. Replaying that order must select the same observation.
    data = json.loads(json.dumps(evidence, sort_keys=True))
    replay = MappingWizard(identity)
    for label, records in data["raw_records"].items():
        replay.capture(label, [InputFrame(**record, identity=identity) for record in records])
    assert json.dumps(replay.build()[1]["selected_samples"], sort_keys=True) == json.dumps(
        evidence["selected_samples"], sort_keys=True
    )


def test_jt07_mapping_loader_rejects_selected_or_claimed_mapping_without_raw_support(tmp_path):
    profile, identity, path = physical_evidence(tmp_path)
    data = json.loads(path.read_text())
    data["selected_samples"]["lx_left"]["axes"][0] = 123
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="RAW_RECORDS_MISMATCH"):
        validate_mapping_evidence(profile, identity)


def test_jt07_qualification_cannot_count_duplicate_trial_under_new_id(tmp_path):
    profile, identity = qualified_profile(tmp_path)
    path = profile.qualification_file
    data = json.loads(open(path).read())
    duplicate = copy.deepcopy(data["trials"][0])
    duplicate["trial_id"] = "copied-under-new-id"
    data["trials"][1] = duplicate
    open(path, "w").write(json.dumps(data))
    with pytest.raises(ValueError, match="DUPLICATE_TRIAL"):
        validate_qualification(profile, identity)


def test_jt07_valid_hash_and_pass_bit_cannot_claim_nonexistent_neutral_response(tmp_path):
    profile, identity = qualified_profile(tmp_path)
    data = json.loads(open(profile.qualification_file).read())
    trial = data["trials"][0]
    trial["raw_records"]["frames"][-1]["axes"][0] = 20000
    trial["raw_records_sha256"] = hashlib.sha256(
        json.dumps(trial["raw_records"], sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    open(profile.qualification_file, "w").write(json.dumps(data))
    with pytest.raises(ValueError, match="RAW_RESPONSE_MISMATCH"):
        validate_qualification(profile, identity)


def test_jt07_unknown_mode_allows_mapping_but_never_qualification(tmp_path):
    profile, identity, _ = physical_evidence(tmp_path)
    original = wizard_from_file(profile, identity)
    unknown = replace(identity, device_mode="operator_reported_unknown_2.4GHz")
    wizard = MappingWizard(unknown)
    for label, records in original.steps.items():
        wizard.capture(label, [replace(frame, identity=unknown) for frame in records])
    mapping, evidence = wizard.build()
    path = tmp_path / "unknown-mapping.json"
    path.write_text(json.dumps(evidence))
    profile = replace(profile, mapping=mapping, mapping_evidence=str(path))
    assert validate_mapping_evidence(profile, unknown)
    with pytest.raises(ValueError, match="Known device mode"):
        QualificationWizard(profile, unknown, 0.2)


def test_jt18_mapping_and_trial_storage_have_explicit_capacity(tmp_path):
    profile, identity, _ = physical_evidence(tmp_path)
    wizard = MappingWizard(identity)
    frame = Frames(real=True).frame({0: 20000})
    with pytest.raises(ValueError, match="1..512"):
        wizard.capture("lx_left", [frame] * 513)
    qualifier = QualificationWizard(profile, identity, 0.2)
    qualifier.trials = [None] * 128
    with pytest.raises(ValueError, match="capacity"):
        qualifier.add_trial("power_off", frame, frame.observed_ns, [], None, operator_attested=True)


@pytest.mark.parametrize("field", ["reader_timeout_s", "input_max_age_s", "poll_timeout_s"])
def test_jt18_finite_but_overflowing_deadline_values_are_rejected(field):
    with pytest.raises(ValueError):
        replace(synthetic(), **{field: 1e300})


@pytest.mark.parametrize(
    "loader,limit", [(read_evidence, 2_000_000), (load_script, 1_000_000), (load_profile, 65_536)]
)
def test_jt18_loaders_limit_actual_file_bytes(tmp_path, loader, limit):
    path = tmp_path / "oversize"
    path.write_bytes(b" " * (limit + 1))
    with pytest.raises(ValueError, match="size limit"):
        loader(path)


def test_jt18_deep_evidence_json_is_rejected_without_recursion_escape(tmp_path):
    path = tmp_path / "deep.json"
    depth = 10000
    path.write_text('{"x":' + "[" * depth + "0" + "]" * depth + "}")
    with pytest.raises(ValueError, match="nesting"):
        read_evidence(path)


def test_jt18_reopen_checks_identity_without_raw_replay_in_reader_thread(tmp_path, monkeypatch):
    from gca_teleop.state_machine import TeleopPipeline

    profile, identity, _ = physical_evidence(tmp_path)
    pipeline = TeleopPipeline(profile)

    def forbidden(*args, **kwargs):
        raise AssertionError(
            "Heavy evidence replay belongs to management, never reader process/tick"
        )

    monkeypatch.setattr("gca_teleop.state_machine.validate_mapping_evidence", forbidden)
    monkeypatch.setattr("gca_teleop.state_machine.validate_qualification", forbidden)
    fixture = Frames(real=True)
    for epoch in ("opened-1", "reopened-2"):
        fixture.epoch = epoch
        frame = fixture.frame()
        result = pipeline.process(frame, fixture.health(), fixture.now, armed=False)
        assert result.reason == "NOT_ARMED_READY"
    fixture.identity = replace(identity, kernel="changed-kernel")
    frame = fixture.frame()
    result = pipeline.process(frame, fixture.health(), fixture.now, armed=True)
    assert result.reason == "MAPPING_EVIDENCE_MISMATCH"


@pytest.mark.parametrize("value", [None, True, float("nan")])
def test_jt04_invalid_trigger_samples_fail_closed(value):
    from gca_teleop.mapping import map_input
    from gca_teleop.model import InputError

    fixture = Frames()
    frame = fixture.frame()
    values = list(frame.axes)
    values[2] = value
    with pytest.raises(InputError, match="INVALID_TRIGGER"):
        map_input(replace(frame, axes=tuple(values)), synthetic())


@pytest.mark.parametrize("failure", ["frozen", "disconnected"])
def test_jt07_qualified_reader_liveness_and_disconnect_are_separate_stop_paths(tmp_path, failure):
    from gca_teleop.state_machine import TeleopPipeline
    from test_input_pipeline import activate

    profile, _ = qualified_profile(tmp_path)
    fixture = Frames(real=True)
    fixture.axes[2] = -32000
    pipeline = TeleopPipeline(profile)
    activate(pipeline, fixture, {0: 20000})
    health = fixture.health()
    if failure == "frozen":
        fixture.now += 100_000_000
        reason = "READER_STALE"
    else:
        health = replace(health, connected=False, reason="DISCONNECTED")
        reason = "DISCONNECTED"
    result = pipeline.tick(health, fixture.now, armed=True, reference_id="r1", session_key="s1")
    assert result.action == "STOP" and result.reason == reason


@pytest.mark.parametrize("event_offset_ns", [-20_000_000, 0, 1])
def test_jt07_fault_response_requires_original_event_lower_bound_after_start(
    tmp_path, event_offset_ns
):
    """Old neutral queued before the fault cannot qualify by arriving later."""
    from gca_teleop.probe import evaluate_qualification_trial

    profile, identity, _ = physical_evidence(tmp_path)
    fixture = Frames(real=True)
    held = fixture.frame({0: 20000, 2: 32767})
    fault_start = held.observed_ns + 50_000_000
    neutral = fixture.frame({0: 0, 2: -32000}, advance=0.07)
    neutral = replace(neutral, raw_event_ns=fault_start + event_offset_ns)
    result = evaluate_qualification_trial(
        profile,
        identity,
        0.1,
        "power_off",
        held,
        fault_start,
        [neutral],
        fixture.health(),
        operator_attested=True,
    )
    if event_offset_ns <= 0:
        assert result["status"] == "FAIL" and result["response_ns"] is None
    else:
        assert result["status"] == "PASS"
        assert result["response_ns"] == neutral.observed_ns


@pytest.mark.parametrize(
    "mutation",
    [
        "same_event_seq",
        "event_seq_backwards",
        "same_observation_seq",
        "observation_time_backwards",
        "event_time_backwards",
        "future_event_time",
        "different_clock",
    ],
)
def test_jt07_qualification_requires_causal_sequence_and_observation_order(tmp_path, mutation):
    from gca_teleop.probe import evaluate_qualification_trial

    profile, identity, _ = physical_evidence(tmp_path)
    fixture = Frames(real=True)
    held = fixture.frame({0: 20000, 2: 32767})
    neutral = fixture.frame({0: 0, 2: -32000}, advance=0.05)
    changes = {
        "same_event_seq": {"event_seq": held.event_seq},
        "event_seq_backwards": {"event_seq": held.event_seq - 1},
        "same_observation_seq": {"observation_seq": held.observation_seq},
        "observation_time_backwards": {"observed_ns": held.observed_ns - 1},
        "event_time_backwards": {"raw_event_ns": held.raw_event_ns - 1},
        "future_event_time": {"raw_event_ns": neutral.observed_ns + 1},
        "different_clock": {"clock_domain_id": "another-boot"},
    }[mutation]
    with pytest.raises(ValueError):
        evaluate_qualification_trial(
            profile,
            identity,
            0.1,
            "power_off",
            held,
            held.observed_ns,
            [replace(neutral, **changes)],
            fixture.health(),
            operator_attested=True,
        )


def test_jt07_right_side_event_cannot_prove_gca_neutral_response(tmp_path):
    from gca_teleop.probe import evaluate_qualification_trial

    profile, identity, _ = physical_evidence(tmp_path)
    fixture = Frames(real=True)
    held = fixture.frame({0: 20000, 2: 32767})
    neutral = fixture.frame({0: 0, 2: -32000, 3: 10000}, advance=0.05)
    neutral = replace(neutral, changed_axes=(3,))
    result = evaluate_qualification_trial(
        profile,
        identity,
        0.1,
        "power_off",
        held,
        held.observed_ns,
        [neutral],
        fixture.health(),
        operator_attested=True,
    )
    assert result["status"] == "FAIL"


def test_jt07_writer_never_reports_pass_for_duplicate_records_loader_rejects(tmp_path):
    profile, identity = qualified_profile(tmp_path)
    data = json.loads(open(profile.qualification_file).read())
    data["trials"][1] = copy.deepcopy(data["trials"][0])
    data["trials"][1]["trial_id"] = "duplicate-new-claim"
    wizard = QualificationWizard(profile, identity, 0.2)
    wizard.trials = data["trials"]
    result = wizard.build()
    assert result["status"] == "FAIL"
    assert result["validation_error"] == "QUALIFICATION_DUPLICATE_TRIAL"


def test_jt07_cli_revalidates_json_output_and_never_writes_invalid_qualified_profile(
    tmp_path, monkeypatch
):
    from gca_teleop import probe

    profile, identity = qualified_profile(tmp_path)
    data = json.loads(open(profile.qualification_file).read())
    data["trials"][1] = copy.deepcopy(data["trials"][0])
    data["trials"][1]["trial_id"] = "duplicate-cli-claim"

    class Source:
        def __init__(self, profile):
            pass

        def open(self):
            return identity

        def close(self):
            pass

    class InvalidWizard:
        CASES = ()

        def __init__(self, *args):
            pass

        def build(self):
            return data

    profile_path = tmp_path / "input.profile.yaml"
    probe._write_profile(profile_path, profile)
    monkeypatch.setattr(probe, "NativeJoydev", Source)
    monkeypatch.setattr(probe, "QualificationWizard", InvalidWizard)
    monkeypatch.setattr(probe, "_collect", lambda *args: [])
    output = tmp_path / "rejected-qualification.json"
    assert (
        probe.main(
            [
                "qualify",
                "--profile",
                str(profile_path),
                "--max-disconnect-s",
                ".2",
                "--output",
                str(output),
            ]
        )
        == 1
    )
    assert json.loads(output.read_text())["status"] == "FAIL"
    assert not output.with_suffix(".profile.yaml").exists()


@pytest.mark.parametrize("field", ["source_id", "input_epoch", "clock_domain_id"])
def test_jt07_missing_provenance_cannot_compare_fault_and_response_times(tmp_path, field):
    from gca_teleop.probe import evaluate_qualification_trial

    profile, identity, _ = physical_evidence(tmp_path)
    fixture = Frames(real=True)
    held = fixture.frame({0: 20000, 2: 32767})
    neutral = fixture.frame({0: 0, 2: -32000}, advance=0.05)
    held, neutral = replace(held, **{field: ""}), replace(neutral, **{field: ""})
    health = fixture.health()
    if field in ("source_id", "input_epoch"):
        health = replace(health, **{field: ""})
    with pytest.raises(ValueError, match="provenance"):
        evaluate_qualification_trial(
            profile,
            identity,
            0.1,
            "power_off",
            held,
            held.observed_ns,
            [neutral],
            health,
            operator_attested=True,
        )


@pytest.mark.parametrize("field", ["driver", "kernel"])
def test_jt07_unknown_driver_or_kernel_cannot_qualify_even_with_consistent_mapping(tmp_path, field):
    profile, identity, _ = physical_evidence(tmp_path)
    recorded = wizard_from_file(profile, identity)
    missing_version = replace(identity, **{field: ""})
    wizard = MappingWizard(missing_version)
    for label, frames in recorded.steps.items():
        wizard.capture(label, [replace(frame, identity=missing_version) for frame in frames])
    _, data = wizard.build()
    output = tmp_path / "mapping-missing-version.json"
    output.write_text(json.dumps(data))
    profile = replace(profile, mapping_evidence=str(output))
    assert validate_mapping_evidence(profile, missing_version)
    with pytest.raises(ValueError, match="Observed driver/kernel"):
        QualificationWizard(profile, missing_version, 0.2)
