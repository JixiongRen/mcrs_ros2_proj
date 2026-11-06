"""Real process regression: verification owns descendants across session changes."""

import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS))
from verify_software import OwnedProcesses, process_identity  # noqa: E402

# A pipe acknowledges that the detached child installed its signal policy. Parent
# exit is then immediate, so retaining only a periodically sampled PPID tree is
# insufficient. The verifier must find the inherited command marker after adoption.
TARGET = r'''
import json, os, signal, subprocess, sys
from pathlib import Path
mode, record = sys.argv[1:]
read_fd, write_fd = os.pipe()
child_source = """
import os,signal,sys
if sys.argv[1] == 'timeout': signal.signal(signal.SIGINT, signal.SIG_IGN)
os.write(int(sys.argv[2]), b'R')
os.close(int(sys.argv[2]))
while True: signal.pause()
"""
child = subprocess.Popen([sys.executable, '-c', child_source, mode, str(write_fd)],
                         pass_fds=(write_fd,), start_new_session=True)
os.close(write_fd)
assert os.read(read_fd, 1) == b'R'
os.close(read_fd)
Path(record).write_text(json.dumps({'parent': os.getpid(), 'child': child.pid}))
if mode == 'timeout': signal.signal(signal.SIGINT, signal.SIG_IGN)
if mode in ('success_leak', 'failure'): raise SystemExit(0 if mode == 'success_leak' else 7)
while True: signal.pause()
'''

WRAPPER = r"""
import sys
from pathlib import Path
sys.path.insert(0, sys.argv[1])
import verify_software as verify
verify.sourced = lambda command, install=None: command
try:
    verify.Verification(Path(sys.argv[2])).run('child', [sys.executable, '-c', sys.argv[3],
        sys.argv[4], sys.argv[5]], deadline_s=.8 if sys.argv[4] == 'timeout' else 8)
except KeyboardInterrupt:
    raise SystemExit(130)
except RuntimeError:
    raise SystemExit(1)
"""


def wait_file(path, process):
    deadline = time.monotonic() + 5
    while not path.exists():
        if process.poll() is not None or time.monotonic() >= deadline:
            raise AssertionError("controlled process handshake missing")
        time.sleep(0.01)
    return json.loads(path.read_text())


def running(pid):
    identity = process_identity(pid)
    return identity is not None and identity.state != "Z"


def cleanup_owned(pid, started):
    identity = process_identity(pid)
    if identity and identity.started == started and identity.state != "Z":
        os.kill(pid, signal.SIGKILL)


@pytest.mark.parametrize("mode", ["failure", "success_leak", "timeout", "interrupt"])
def test_verification_cleans_only_its_detached_descendants_and_records_failure(tmp_path, mode):
    unrelated = subprocess.Popen(
        [sys.executable, "-c", "import signal; signal.pause()"], start_new_session=True
    )
    out, record = tmp_path / "evidence", tmp_path / "pids.json"
    process = subprocess.Popen(
        [sys.executable, "-c", WRAPPER, str(TOOLS), str(out), TARGET, mode, str(record)],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    identities = []
    try:
        pids = wait_file(record, process)
        identities = [i for pid in pids.values() if (i := process_identity(pid))]
        if mode == "interrupt":
            os.kill(process.pid, signal.SIGINT)
        stdout, _ = process.communicate(timeout=10)
        assert process.returncode == (130 if mode == "interrupt" else 1), stdout
        entry = json.loads((out / "report.json").read_text())["commands"][0]
        assert entry["cleanup"]["complete"], entry
        assert entry["cleanup"]["found_live"] >= 1
        assert entry["cleanup"]["remaining"] == []
        assert entry["cleanup"]["errors"] == []
        assert not any(running(pid) for pid in pids.values())
        assert unrelated.poll() is None
        assert unrelated.pid not in {item["group"] for item in entry["cleanup"]["signals"]}
        if mode == "timeout":
            assert entry["timeout"]
            assert any(s["signal"] == "SIGKILL" for s in entry["cleanup"]["signals"])
        elif mode == "interrupt":
            assert entry["interrupted"]
        elif mode == "failure":
            assert entry["exit_code"] == 7
        else:
            # A successful parent which leaks a child is still a failed check.
            assert entry["exit_code"] == 0
    finally:
        for identity in identities:
            cleanup_owned(identity.pid, identity.started)
        if process.poll() is None:
            process.kill()
        process.communicate(timeout=3)
        unrelated.terminate()
        unrelated.wait(timeout=3)


def test_success_without_live_descendants_passes(tmp_path, monkeypatch):
    import verify_software as verify

    monkeypatch.setattr(verify, "sourced", lambda command, install=None: command)
    verification = verify.Verification(tmp_path)
    verification.run("clean", [sys.executable, "-c", "print('complete')"], deadline_s=3)
    result = verification.report["commands"][0]
    assert result["exit_code"] == 0
    assert result["cleanup"] == {
        "found_live": 0,
        "signals": [],
        "remaining": [],
        "errors": [],
        "complete": True,
    }


def test_reused_pid_identity_cannot_authorize_signal(monkeypatch):
    import verify_software as verify

    owned = OwnedProcesses(os.getpid(), "test-nonmatching-marker")
    identity = process_identity(os.getpid())
    old = verify.ProcessIdentity(identity.pid, identity.started - 1, 0, identity.group, "S")
    owned.known = {(old.pid, old.started): old}
    monkeypatch.setattr(owned, "capture", lambda: [old])
    signals = []
    monkeypatch.setattr(os, "killpg", lambda *args: signals.append(args))
    result = owned.cleanup(grace_s=0, kill_s=0)
    assert not signals
    assert result["remaining"] == [identity.pid] and not result["complete"]


def test_truncated_snapshot_keeps_known_identity_for_best_effort_cleanup(monkeypatch):
    owned = OwnedProcesses(os.getpid(), "test-nonmatching-marker")
    identity = process_identity(os.getpid())
    monkeypatch.setattr(owned, "scan_budget_s", 0)
    assert identity in owned.capture()
    assert "PROCESS_SCAN_CAPACITY_EXCEEDED" in owned.errors
