"""Qualification CLI failure persistence with mocked input; never open devices."""

import json

from gca_teleop import probe
from gca_teleop.config import load_profile, validate_qualification
from test_input_pipeline import Frames, physical_evidence


class QualificationRun:
    def __init__(self, tmp_path, monkeypatch, failure=""):
        self.profile, self.identity, _ = physical_evidence(tmp_path)
        self.fixture = Frames(real=True)
        self.fixture.axes[2] = -32000
        self.failure = failure
        self.output = tmp_path / "qualification.json"
        self.trial_count = 0
        self.attestation_count = 0
        self.capture_phase = 0
        self.closed = False
        self.partial_poll_count = 0
        self.original_collect = probe._collect
        monkeypatch.setattr(probe, "load_profile", lambda _: self.profile)
        monkeypatch.setattr(probe, "NativeJoydev", lambda _: self)
        monkeypatch.setattr(probe, "monotonic_ns", lambda: self.fixture.now)
        monkeypatch.setattr(probe, "_collect", self.collect)
        monkeypatch.setattr("builtins.input", self.operator)

    def open(self):
        self.closed = False
        return self.identity

    def close(self):
        self.closed = True

    def health(self):
        return self.fixture.health()

    def operator(self, prompt):
        if prompt.startswith("Type CONFIRM"):
            self.attestation_count += 1
            return "NO" if self.failure == "operator" and self.trial_count == 2 else "CONFIRM"
        self.trial_count += 1
        self.capture_phase = 0
        return ""

    def collect(self, source, seconds, *, frames=None):
        if frames is None:
            frames = []
        if self.trial_count == 0:
            return frames  # Initial reader setup, before the first operator cue.
        self.capture_phase += 1
        if self.capture_phase == 1:
            frames.append(self.fixture.frame(real_event=False))
        elif self.capture_phase == 2:
            frames.append(self.fixture.frame({0: 20000, 2: 32767}))
        elif self.failure == "capture" and self.trial_count == 2:
            # Exercise the actual collector: its first poll succeeds and its
            # second raises, so the caller must retain the acquired frame.
            return self.original_collect(source, seconds, frames=frames)
        elif self.failure == "timeout" and self.trial_count == 2:
            frames.append(self.fixture.frame(real_event=False, advance=0.25))
        else:
            frames.append(self.fixture.frame({0: 0, 2: -32000}, advance=0.05))
        return frames

    def poll(self):
        self.partial_poll_count += 1
        if self.partial_poll_count == 1:
            return [self.fixture.frame({0: 21000})]
        raise OSError("injected capture read failure")

    def run(self):
        return probe.main(
            [
                "qualify",
                "--profile",
                "unused.yaml",
                "--output",
                str(self.output),
                "--max-disconnect-s",
                "0.2",
                "--capture-s",
                "0.1",
            ]
        )

    def result(self):
        return json.loads(self.output.read_text())

    def assert_stopped(self, capsys):
        assert self.closed
        assert self.trial_count == 2
        assert not self.output.with_suffix(".profile.yaml").exists()
        assert capsys.readouterr().out.splitlines()[-1] == "FAIL"


def test_failed_trial_stops_immediately_and_keeps_prior_pass(tmp_path, monkeypatch, capsys):
    run = QualificationRun(tmp_path, monkeypatch, failure="timeout")
    assert run.run() == 1
    result = run.result()
    assert result["status"] == "FAIL"
    assert result["completed_trial_count"] == 2
    assert [trial["status"] for trial in result["trials"]] == ["PASS", "FAIL"]
    assert result["trials"][1]["response_ns"] is None
    assert "incomplete_trial" not in result
    assert run.attestation_count == 2
    run.assert_stopped(capsys)


def test_operator_refusal_is_incomplete_evidence_and_stops(tmp_path, monkeypatch, capsys):
    run = QualificationRun(tmp_path, monkeypatch, failure="operator")
    assert run.run() == 1
    result = run.result()
    assert result["status"] == "FAIL" and result["completed_trial_count"] == 1
    assert len(result["trials"]) == 1 and result["trials"][0]["status"] == "PASS"
    incomplete = result["incomplete_trial"]
    assert (incomplete["case"], incomplete["trial_number"]) == ("power_off", 2)
    assert incomplete["stage"] == "evaluate"
    assert "Operator must attest" in incomplete["error"]
    raw = incomplete["raw_records"]
    assert raw["operator_attested"] is False
    assert len(raw["response_frames"]) == 1 and "final_health" in raw
    assert "response_ns" not in incomplete and "held_nonzero" not in incomplete
    run.assert_stopped(capsys)


def test_capture_exception_preserves_partial_frames_without_inventing_result(
    tmp_path, monkeypatch, capsys
):
    run = QualificationRun(tmp_path, monkeypatch, failure="capture")
    assert run.run() == 1
    result = run.result()
    assert result["status"] == "FAIL" and result["completed_trial_count"] == 1
    incomplete = result["incomplete_trial"]
    assert incomplete["stage"] == "response_capture"
    assert incomplete["error"] == "OSError: injected capture read failure"
    raw = incomplete["raw_records"]
    assert len(raw["held_frames"]) == 2 and len(raw["response_frames"]) == 1
    assert raw["response_frames"][0]["axes"][0] == 21000
    assert "start_ns" in raw and "last_health" in raw
    assert "final_health" not in raw and "operator_attested" not in raw
    assert run.attestation_count == 1
    run.assert_stopped(capsys)


def test_nine_successful_trials_still_write_valid_qualified_profile(tmp_path, monkeypatch, capsys):
    run = QualificationRun(tmp_path, monkeypatch)
    assert run.run() == 0
    result = run.result()
    assert result["status"] == "PASS" and len(result["trials"]) == 9
    assert run.trial_count == run.attestation_count == 9 and run.closed
    profile = load_profile(run.output.with_suffix(".profile.yaml"))
    assert profile.mode == "qualified_state"
    assert validate_qualification(profile, run.identity) == result["record_id"]
    assert capsys.readouterr().out.splitlines()[-1] == "PASS"
