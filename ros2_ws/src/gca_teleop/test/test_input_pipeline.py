"""JT01-08: source-level pure teleop acceptance; no real hardware claim."""

import json
from dataclasses import replace
from pathlib import Path

import pytest
from gca.domain.manual import QualityTracker, host_clock_id
from gca_teleop.config import (
    DeviceSelector,
    load_profile,
    validate_mapping_evidence,
    validate_qualification,
)
from gca_teleop.mapping import map_input
from gca_teleop.model import InputError, InputFrame, InputHealth, InputIdentity
from gca_teleop.probe import MappingWizard, QualificationWizard
from gca_teleop.state_machine import TeleopPipeline

CONFIG = Path(__file__).parents[1] / "config"


def synthetic():
    return load_profile(CONFIG / "synthetic.yaml")


def identity(real=False):
    return InputIdentity(
        "js0" if real else "scripted",
        "fixture",
        0x20BC,
        0x506D,
        3,
        1,
        tuple(range(8)),
        tuple(range(16)),
        "usb-fixture",
        "",
        "test-driver",
        "test-kernel",
        "test-mode",
    )


class Frames:
    def __init__(self, real=False):
        self.identity = identity(real)
        self.real = real
        self.axes = [0, 0, -32768, 0, 0, 0, 0, 0]
        self.buttons = [0] * 16
        self.now = 10_000_000_000
        self.seq = 0
        self.obseq = 0
        self.epoch = "epoch-1"
        self.raw = 0

    def frame(self, axes=None, buttons=None, *, advance=0.01, initialized=True, real_event=True):
        self.now += int(advance * 1e9)
        changed_axes, changed_buttons = [], []
        for changes, values, changed in (
            (axes or {}, self.axes, changed_axes),
            (buttons or {}, self.buttons, changed_buttons),
        ):
            for key, value in changes.items():
                if values[key] != value:
                    values[key] = value
                    changed.append(key)
        self.obseq += 1
        if real_event:
            self.seq += 1
            self.raw = self.now - 1_000_000
        return InputFrame(
            "fixture",
            self.epoch,
            self.seq,
            self.raw,
            self.obseq,
            self.now,
            host_clock_id(),
            tuple(self.axes),
            tuple(self.buttons),
            tuple(changed_axes),
            tuple(changed_buttons),
            initialized,
            not self.real,
            identity=self.identity,
            source_kind="JOYDEV" if self.real else "SCRIPT_SAMPLE",
        )

    def health(self, **changes):
        return replace(
            InputHealth(True, True, self.now, "fixture", self.epoch, identity=self.identity),
            **changes,
        )


def process(pipeline, fixture, **kwargs):
    frame = fixture.frame(**kwargs)
    return pipeline.process(
        frame, fixture.health(), fixture.now, armed=True, reference_id="r1", session_key="s1"
    )


def activate(pipeline, fixture, axes=None):
    assert process(pipeline, fixture).gate == "READY"
    result = process(pipeline, fixture, axes={2: 32767, **(axes or {})})
    assert result.action == "JOG"
    return result


def physical_evidence(tmp_path):
    fixture = Frames(real=True)
    wizard = MappingWizard(fixture.identity)
    steps = [
        ("lx_left", {0: -32768}, {}),
        ("lx_center", {0: 0}, {}),
        ("lx_right", {0: 32767}, {}),
        ("ly_up", {0: 0, 1: -32768}, {}),
        ("ly_center", {1: 0}, {}),
        ("ly_down", {1: 32767}, {}),
        ("lt_released", {1: 0, 2: -32000}, {}),
        ("lt_pressed", {2: 32767}, {}),
        ("b_pressed", {}, {1: 1}),
        ("b_released", {}, {1: 0}),
        ("rt_pressed", {3: 32767}, {}),
        ("rb_pressed", {}, {5: 1}),
        ("rx_right", {4: 32767}, {}),
        ("ry_up", {5: -32768}, {}),
        ("rs_pressed", {}, {9: 1}),
    ]
    for label, axes, buttons in steps:
        wizard.capture(label, [fixture.frame(axes, buttons)])
    mapping, evidence = wizard.build()
    path = tmp_path / "mapping.json"
    path.write_text(json.dumps(evidence))
    profile = replace(
        synthetic(),
        source="joydev",
        mapping=mapping,
        mapping_evidence=str(path),
        script_file="",
        device=DeviceSelector(device_mode="test-mode"),
    )
    return profile, fixture.identity, path


def test_jt02_independent_mapping_and_full_diagonal():
    fixture = Frames()
    config = synthetic()
    frame = fixture.frame({0: 32767, 1: -32768, 2: 0})
    value = map_input(frame, config)
    assert value.lx == value.ly == 1
    pipeline = TeleopPipeline(config)
    fixture = Frames()
    result = activate(pipeline, fixture, {0: 32767, 1: -32768})
    assert result.linear_mm_s == 2 and result.rotation_deg_s == 30
    assert result.deadman and result.quality.source_kind == "SCRIPT_SAMPLE"


def test_jt02_deadzone_response_and_polarity_exactly_once():
    fixture = Frames()
    assert map_input(fixture.frame({0: 1000, 1: -1000}), synthetic()).centered
    first = map_input(fixture.frame({0: 16000, 1: -16000}), synthetic())
    assert 0 < first.lx < 0.5 and 0 < first.ly < 0.5
    reverse = replace(
        synthetic(),
        mapping=replace(synthetic().mapping, ly=replace(synthetic().mapping.ly, sign=1)),
    )
    assert map_input(fixture.frame({1: -32768}), reverse).ly == -1


def test_jt04_lt_hysteresis_is_boolean_not_speed_scale():
    fixture = Frames()
    pipeline = TeleopPipeline(synthetic())
    process(pipeline, fixture)
    result = process(pipeline, fixture, axes={0: 32767, 2: 10000})
    assert result.action == "JOG" and result.rotation_deg_s == 30
    result = process(pipeline, fixture, axes={2: 0})
    assert result.deadman and result.rotation_deg_s == 30
    assert process(pipeline, fixture, axes={2: -20000}).reason == "DEADMAN_RELEASED"


@pytest.mark.parametrize("reason", ["b", "expiry", "context"])
def test_jt04_jt05_stop_never_invents_release_or_resumes_held_lt(reason):
    fixture = Frames()
    pipeline = TeleopPipeline(synthetic())
    activate(pipeline, fixture, {0: 20000})
    if reason == "b":
        assert process(pipeline, fixture, buttons={1: 1}).reason == "B_STOP"
        process(pipeline, fixture, buttons={1: 0})
    elif reason == "expiry":
        fixture.now += 200_000_000
        assert (
            pipeline.tick(
                fixture.health(), fixture.now, armed=True, reference_id="r1", session_key="s1"
            ).action
            == "STOP"
        )
    else:
        assert (
            pipeline.tick(
                fixture.health(), fixture.now, armed=True, reference_id="r2", session_key="s1"
            ).action
            == "STOP"
        )
        # Restore stable session context, without releasing LT.
        process(pipeline, fixture)
    assert process(pipeline, fixture, axes={0: 0}).gate == "WAIT_NEUTRAL"
    assert (
        pipeline.tick(
            fixture.health(), fixture.now, armed=True, reference_id="r1", session_key="s1"
        ).action
        != "JOG"
    )
    assert process(pipeline, fixture, axes={0: 20000}).action != "JOG"
    process(pipeline, fixture, axes={0: 0, 2: -32768})
    assert process(pipeline, fixture, axes={0: 20000, 2: 32767}).action == "JOG"


def test_jt03_reserved_controls_do_not_change_intent_or_real_event_age(tmp_path):
    config, _, _ = physical_evidence(tmp_path)
    fixture = Frames(real=True)
    fixture.axes[2] = -32000
    pipeline = TeleopPipeline(config)
    first = activate(pipeline, fixture, {0: 20000})
    for axis in (3, 4, 5, 6, 7):
        result = process(pipeline, fixture, axes={axis: 10000})
        assert result.linear_mm_s == first.linear_mm_s
        assert result.rotation_deg_s == first.rotation_deg_s
        assert result.quality.event_ns == first.quality.event_ns
        assert result.quality.valid_until_ns == first.quality.valid_until_ns
    for button in (5, 9, 8, 0, 2, 3):
        result = process(pipeline, fixture, buttons={button: 1})
        assert result.quality.valid_until_ns == first.quality.valid_until_ns
    fixture.now = first.quality.valid_until_ns
    assert (
        pipeline.tick(
            fixture.health(), fixture.now, armed=True, reference_id="r1", session_key="s1"
        ).reason
        == "GCA_INPUT_EXPIRED"
    )


def test_jt06_deadzone_noise_does_not_renew_age(tmp_path):
    config, _, _ = physical_evidence(tmp_path)
    fixture = Frames(real=True)
    fixture.axes[2] = -32000
    pipeline = TeleopPipeline(config)
    first = activate(pipeline, fixture)
    result = process(pipeline, fixture, axes={0: 100, 1: -100})
    assert result.quality.valid_until_ns == first.quality.valid_until_ns
    assert result.rotation_deg_s == result.linear_mm_s == 0


def test_jt04_center_is_zero_velocity_not_stop():
    fixture = Frames()
    pipeline = TeleopPipeline(synthetic())
    activate(pipeline, fixture, {0: 20000, 1: -20000})
    result = process(pipeline, fixture, axes={0: 0, 1: 0})
    assert result.action == "JOG" and result.deadman
    assert result.linear_mm_s == result.rotation_deg_s == 0


def test_jt05_reader_frozen_and_disconnect_revoke():
    fixture = Frames()
    pipeline = TeleopPipeline(synthetic())
    activate(pipeline, fixture, {0: 20000})
    health = fixture.health()
    fixture.now += 100_000_000
    assert (
        pipeline.tick(health, fixture.now, armed=True, reference_id="r1", session_key="s1").reason
        == "READER_STALE"
    )
    assert (
        pipeline.tick(
            replace(health, connected=False, reason="DISCONNECTED"),
            fixture.now,
            armed=True,
            reference_id="r1",
            session_key="s1",
        ).action
        == "STOP"
    )


def test_jt07_unverified_mapping_and_unqualified_hardware_locked():
    fixture = Frames(real=True)
    config = load_profile(CONFIG / "beitong_unverified.yaml")
    pipeline = TeleopPipeline(config)
    result = process(pipeline, fixture)
    assert result.reason == "MAPPING_UNVERIFIED"
    fixture = Frames()
    pipeline = TeleopPipeline(synthetic())
    frame = fixture.frame()
    assert (
        pipeline.process(frame, fixture.health(), fixture.now, armed=True, hardware=True).reason
        == "HARDWARE_INPUT_NOT_QUALIFIED"
    )


def test_jt05_epoch_init_and_config_transition_need_new_neutral():
    fixture = Frames()
    pipeline = TeleopPipeline(synthetic())
    activate(pipeline, fixture, {0: 20000})
    fixture.epoch = "epoch-2"
    assert process(pipeline, fixture).reason == "INPUT_RESYNC"
    assert process(pipeline, fixture, axes={0: 0}).action != "JOG"
    with pytest.raises(InputError, match="CONFIG_REQUIRES_STOPPED_DISARMED"):
        pipeline.set_config(synthetic(), stopped=False, disarmed=True)
    pipeline.set_config(synthetic(), stopped=True, disarmed=True)
    assert process(pipeline, fixture).action != "JOG"


def test_jt08_script_quality_passes_core_and_tick_cannot_refresh():
    fixture = Frames()
    pipeline = TeleopPipeline(synthetic())
    result = activate(pipeline, fixture, {0: 20000})
    tracker = QualityTracker()
    assert tracker.validate(result.quality, fixture.now)
    again = pipeline.tick(
        fixture.health(), fixture.now + 1, armed=True, reference_id="r1", session_key="s1"
    )
    assert again.quality == result.quality


def test_mapping_requires_observed_events_and_digest(tmp_path):
    config, device, path = physical_evidence(tmp_path)
    assert validate_mapping_evidence(config, device)
    data = json.loads(path.read_text())
    data["raw_records"]["lx_left"][0]["axes"][0] = 123
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="HASH_MISMATCH"):
        validate_mapping_evidence(config, device)
    with pytest.raises(ValueError, match="No physical change"):
        fixture = Frames(real=True)
        MappingWizard(fixture.identity).capture("lx_left", [fixture.frame(real_event=False)])


def qualified_profile(tmp_path):
    config, device, _ = physical_evidence(tmp_path)
    wizard = QualificationWizard(config, device, 0.2)
    for case in wizard.CASES:
        for trial_index in range(3):
            fixture = Frames(real=True)
            fixture.epoch = f"trial-{case}-{trial_index}"
            fixture.axes[2] = -32000
            held = fixture.frame({0: 20000, 2: 32767})
            start = held.observed_ns
            neutral = fixture.frame({0: 0, 2: -32000}, advance=0.05)
            assert (
                wizard.add_trial(
                    case, held, start, [neutral], fixture.health(), operator_attested=True
                )["status"]
                == "PASS"
            )
    path = tmp_path / "qualification.json"
    path.write_text(json.dumps(wizard.build()))
    return replace(config, mode="qualified_state", qualification_file=str(path)), device


def test_qualified_state_observation_progress_without_fake_radio_event(tmp_path):
    config, device = qualified_profile(tmp_path)
    assert validate_qualification(config, device)
    fixture = Frames(real=True)
    fixture.axes[2] = -32000
    pipeline = TeleopPipeline(config)
    first = activate(pipeline, fixture, {0: 20000})
    result = process(pipeline, fixture, real_event=False, advance=0.08)
    assert result.quality.source_kind == result.quality.link_evidence == "QUALIFIED_STATE_CONTRACT"
    assert result.quality.event_ns == first.quality.event_ns
    assert result.quality.observation_ns > first.quality.observation_ns
    assert result.quality.valid_until_ns > first.quality.valid_until_ns
    tracker = QualityTracker(backend="hardware")
    tracker.validate(first.quality, first.quality.observation_ns)
    tracker.validate(result.quality, fixture.now)


def test_qualification_incomplete_synthetic_late_and_changed_identity_fail(tmp_path):
    config, device, _ = physical_evidence(tmp_path)
    wizard = QualificationWizard(config, device, 0.03)
    fixture = Frames(real=True)
    held = fixture.frame({0: 20000, 2: 32767})
    late = fixture.frame({0: 0, 2: -32000}, advance=0.1)
    assert (
        wizard.add_trial(
            "power_off", held, held.observed_ns, [late], fixture.health(), operator_attested=True
        )["status"]
        == "FAIL"
    )
    assert wizard.build()["status"] == "FAIL"
    config, device = qualified_profile(tmp_path)
    with pytest.raises(ValueError, match="MISMATCH"):
        validate_qualification(config, replace(device, kernel="new-kernel"))


@pytest.mark.parametrize(
    "field,value",
    [
        ("deadzone", float("nan")),
        ("seed", True),
        ("max_poll_events", 0),
        ("mode", "auto"),
        ("lt_press", 0.3),
        ("max_epoch_s", 2**32),
    ],
)
def test_config_rejects_nonfinite_bool_unknown_and_unbounded(field, value):
    with pytest.raises(ValueError):
        replace(synthetic(), **{field: value})


def test_jt02_mapping_monotonic_continuous_and_odd_normalized_amplitude():
    config = synthetic()
    fixture = Frames()
    values = [map_input(fixture.frame({0: x}), config).lx for x in range(0, 32768, 512)]
    assert values == sorted(values) and values[0] == 0
    for fraction in (0.08, 0.1, 0.3, 0.7, 1):
        positive = map_input(fixture.frame({0: round(32767 * fraction)}), config).lx
        negative = map_input(fixture.frame({0: -round(32768 * fraction)}), config).lx
        assert abs(positive + negative) < 0.0001
    threshold = round(config.deadzone * 32767)
    assert abs(map_input(fixture.frame({0: threshold + 1}), config).lx) < 0.000001


def test_jt04_b_wins_simultaneous_new_lt_and_axes():
    fixture = Frames()
    pipeline = TeleopPipeline(synthetic())
    assert process(pipeline, fixture).gate == "READY"
    result = process(pipeline, fixture, axes={0: 20000, 2: 32767}, buttons={1: 1})
    assert result.action == "STOP" and result.reason == "B_STOP"
    assert process(pipeline, fixture, buttons={1: 0}).action != "JOG"


def test_jt05_initially_held_or_deflected_never_enables_after_arm():
    fixture = Frames()
    pipeline = TeleopPipeline(synthetic())
    assert process(pipeline, fixture, axes={2: 32767, 0: 20000}).action != "JOG"
    assert process(pipeline, fixture, axes={0: 0}).action != "JOG"
    assert process(pipeline, fixture, axes={0: 20000}).action != "JOG"
    assert process(pipeline, fixture, axes={2: -32768}).action != "JOG"
    assert process(pipeline, fixture, axes={0: 0}).gate == "READY"
    assert process(pipeline, fixture, axes={2: 32767}).action == "JOG"


def test_jt05_arm_context_changed_on_input_thread_stops_previous_active():
    fixture = Frames()
    pipeline = TeleopPipeline(synthetic())
    activate(pipeline, fixture, {0: 20000})
    frame = fixture.frame()
    result = pipeline.process(
        frame,
        fixture.health(),
        fixture.now,
        armed=True,
        reference_id="changed-reference",
        session_key="s1",
    )
    assert result.action == "STOP" and result.reason == "CONTEXT_CHANGED"


def test_jt06_static_event_age_expires_despite_reader_progress(tmp_path):
    profile, _, _ = physical_evidence(tmp_path)
    fixture = Frames(real=True)
    fixture.axes[2] = -32000
    pipeline = TeleopPipeline(profile)
    first = activate(pipeline, fixture, {0: 20000})
    for _ in range(2):
        frame = fixture.frame(real_event=False, advance=0.06)
        result = pipeline.process(
            frame, fixture.health(), fixture.now, armed=True, reference_id="r1", session_key="s1"
        )
        assert result.action == "JOG"
        assert result.quality.valid_until_ns == first.quality.valid_until_ns
    assert process(pipeline, fixture, real_event=False, advance=0.06).reason == "GCA_INPUT_EXPIRED"


def test_jt07_qualification_bad_file_locks_without_event_age_fallback(tmp_path):
    config, _, _ = physical_evidence(tmp_path)
    pipeline = TeleopPipeline(
        replace(config, mode="qualified_state", qualification_file=str(tmp_path / "missing.json"))
    )
    fixture = Frames(real=True)
    result = process(pipeline, fixture)
    assert result.action == "STOP" and result.reason == "QUALIFICATION_MISMATCH"
    assert process(pipeline, fixture, axes={2: 32767, 0: 20000}).action != "JOG"


def test_jt07_qualified_identity_change_mid_epoch_revokes(tmp_path):
    config, _ = qualified_profile(tmp_path)
    fixture = Frames(real=True)
    fixture.axes[2] = -32000
    pipeline = TeleopPipeline(config)
    activate(pipeline, fixture, {0: 20000})
    fixture.identity = replace(fixture.identity, driver="changed-driver")
    result = process(pipeline, fixture, real_event=False)
    assert result.action == "STOP" and result.reason == "MAPPING_EVIDENCE_MISMATCH"


def test_jt08_invalid_time_and_replayed_frame_stop():
    fixture = Frames()
    pipeline = TeleopPipeline(synthetic())
    activate(pipeline, fixture, {0: 20000})
    frame = fixture.frame()
    for invalid in (
        replace(frame, observed_ns=fixture.now + 1),
        replace(frame, raw_event_ns=float("nan")),
        replace(frame, event_seq=True),
    ):
        result = pipeline.process(
            invalid, fixture.health(), fixture.now, armed=True, reference_id="r1", session_key="s1"
        )
        assert result.action in ("STOP", "NONE") and result.reason == "INPUT_FRAME_INVALID"
    fixture = Frames()
    pipeline = TeleopPipeline(synthetic())
    old = fixture.frame()
    pipeline.process(
        old, fixture.health(), fixture.now, armed=True, reference_id="r1", session_key="s1"
    )
    activate_frame = fixture.frame({2: 32767, 0: 20000})
    assert (
        pipeline.process(
            activate_frame,
            fixture.health(),
            fixture.now,
            armed=True,
            reference_id="r1",
            session_key="s1",
        ).action
        == "JOG"
    )
    assert (
        pipeline.process(
            old, fixture.health(), fixture.now, armed=True, reference_id="r1", session_key="s1"
        ).reason
        == "INPUT_FRAME_REPLAY"
    )


def test_evidence_is_loaded_outside_process_and_tick(tmp_path, monkeypatch):
    config, _, _ = physical_evidence(tmp_path)
    pipeline = TeleopPipeline(config)
    monkeypatch.setattr(
        "gca_teleop.state_machine.read_evidence",
        lambda _: (_ for _ in ()).throw(AssertionError("tick file IO")),
    )
    fixture = Frames(real=True)
    fixture.axes[2] = -32000
    activate(pipeline, fixture)
    fixture.epoch = "reopen"
    assert process(pipeline, fixture).action != "JOG"


def test_mapping_cli_generates_loadable_inactive_profile(tmp_path, monkeypatch):
    from gca_teleop import probe
    from gca_teleop.config import profile_from_dict

    config, device, evidence_path = physical_evidence(tmp_path)
    evidence = json.loads(evidence_path.read_text())
    profile_path = tmp_path / "start.yaml"
    probe._write_profile(
        profile_path, replace(config, mapping=profile_from_dict({}).mapping, mapping_evidence="")
    )
    frames_by_label = {
        label: [InputFrame(**record, identity=device) for record in records]
        for label, records in evidence["raw_records"].items()
    }
    queue = [[]]
    for label in MappingWizard.LABELS:
        queue.extend([[], frames_by_label[label]])

    class Source:
        def __init__(self, profile):
            pass

        def open(self):
            return device

        def close(self):
            pass

    monkeypatch.setattr(probe, "NativeJoydev", Source)
    monkeypatch.setattr(probe, "_collect", lambda *a: queue.pop(0))
    monkeypatch.setattr("builtins.input", lambda *a: "")
    target = tmp_path / "observed.json"
    assert probe.main(["mapping", "--profile", str(profile_path), "--output", str(target)]) == 0
    generated = load_profile(target.with_suffix(".profile.yaml"))
    assert generated.mode == "event_age" and not generated.qualification_file
    assert validate_mapping_evidence(generated, device)
    assert not queue


def test_generated_profile_and_evidence_refuse_overwrite(tmp_path):
    from gca_teleop import probe

    path = tmp_path / "evidence.json"
    probe._write(path, {"status": "NOT_RUN"})
    with pytest.raises(FileExistsError):
        probe._write(path, {"status": "PASS"})
    profile_path = tmp_path / "profile.yaml"
    probe._write_profile(profile_path, synthetic())
    assert load_profile(profile_path).mapping == synthetic().mapping
    with pytest.raises(FileExistsError):
        probe._write_profile(profile_path, synthetic())


def test_qualify_cli_requires_attestation_and_generates_inactive_contract(tmp_path, monkeypatch):
    from gca_teleop import probe

    config, device, _ = physical_evidence(tmp_path)
    profile_path = tmp_path / "mapped.profile.yaml"
    probe._write_profile(profile_path, config)
    fixture = Frames(real=True)
    held = fixture.frame({0: 20000, 2: 32767})
    neutral = fixture.frame({0: 0, 2: -32000}, advance=0.05)
    queue = [[]]
    for index in range(9):
        attempt = Frames(real=True)
        attempt.now += index * 1_000_000_000
        attempt.epoch = f"cli-trial-{index}"
        h = attempt.frame({0: 20000, 2: 32767})
        n = attempt.frame({0: 0, 2: -32000}, advance=0.05)
        queue.extend([[], [h], [n]])

    class Source:
        def __init__(self, profile):
            pass

        def open(self):
            return device

        def close(self):
            pass

        def health(self):
            frame = current[0]
            return InputHealth(
                True, True, frame.observed_ns, frame.source_id, frame.input_epoch, identity=device
            )

    current = [held]

    def collect(*args, frames=None):
        batch = queue.pop(0)
        if batch:
            current[0] = batch[-1]
        if frames is None:
            return batch
        frames.extend(batch)
        return frames

    monkeypatch.setattr(probe, "NativeJoydev", Source)
    monkeypatch.setattr(probe, "_collect", collect)
    monkeypatch.setattr(probe, "monotonic_ns", lambda: current[0].observed_ns)
    prompts = []

    def confirm(prompt):
        prompts.append(prompt)
        return "CONFIRM" if "CONFIRM" in prompt else ""

    monkeypatch.setattr("builtins.input", confirm)
    output = tmp_path / "qualified.json"
    assert (
        probe.main(
            [
                "qualify",
                "--profile",
                str(profile_path),
                "--output",
                str(output),
                "--max-disconnect-s",
                ".2",
            ]
        )
        == 0
    )
    generated = load_profile(output.with_suffix(".profile.yaml"))
    assert generated.mode == "qualified_state" and validate_qualification(generated, device)
    assert len([p for p in prompts if "CONFIRM" in p]) == 9
    assert not queue
    with pytest.raises(ValueError, match="attest"):
        QualificationWizard(config, device, 0.2).add_trial(
            "power_off",
            held,
            held.observed_ns,
            [neutral],
            fixture.health(),
            operator_attested=False,
        )


def test_hardware_arm_precondition_requires_current_qualified_reader(tmp_path):
    fixture = Frames()
    pipeline = TeleopPipeline(synthetic())
    process(pipeline, fixture)
    assert (
        pipeline.hardware_arm_error(fixture.health(), fixture.now) == "HARDWARE_INPUT_NOT_QUALIFIED"
    )
    config, _ = qualified_profile(tmp_path)
    fixture = Frames(real=True)
    fixture.axes[2] = -32000
    pipeline = TeleopPipeline(config)
    frame = fixture.frame()
    pipeline.process(frame, fixture.health(), fixture.now, armed=False, hardware=True)
    assert pipeline.hardware_arm_error(fixture.health(), fixture.now) == ""
    assert (
        pipeline.hardware_arm_error(fixture.health(connected=False), fixture.now)
        == "INPUT_NOT_READY"
    )
    assert (
        pipeline.hardware_arm_error(fixture.health(), fixture.now + 100_000_000) == "READER_STALE"
    )
