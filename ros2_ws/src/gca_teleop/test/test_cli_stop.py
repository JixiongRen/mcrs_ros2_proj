from gca_teleop import cli


def test_stop_wait_requires_this_stop_confirmation_not_cached_progress(monkeypatch, capsys):
    calls = []
    monkeypatch.setattr(
        cli,
        "request",
        lambda *a, **kw: {
            "accepted": True,
            "stop_progress": "CONFIRMED",
            "stop_confirmed": False,
        },
    )
    observations = iter(
        [
            {"accepted": True, "stop_progress": "CONFIRMED", "stop_confirmed": False},
            {"accepted": True, "stop_progress": "CONFIRMED", "stop_confirmed": True},
        ]
    )

    def observe(*args, **kwargs):
        calls.append(kwargs)
        return next(observations)

    monkeypatch.setattr(cli, "call", observe)
    monkeypatch.setattr(cli.time, "sleep", lambda duration: None)
    assert cli.main(["stop", "--wait"]) == 0
    assert len(calls) == 2
    assert all(c["priority"] for c in calls)
    assert '"stop_confirmed": true' in capsys.readouterr().out
