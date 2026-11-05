"""Finite-wave evidence and interruption tests using real adapter logic (T06)."""

from dataclasses import replace
from types import SimpleNamespace

import pytest
from gca.devices.common import DeviceError, PigpioConfig
from gca.devices.pigpio_engine import PigpioPulseEngine
from gca.domain.models import PulseSegment


class Client:
    def __init__(self):
        self.connected, self.busy, self.wave_id = True, 0, 7
        self.waveform = []
        self.deleted = []
        self.writes = []
        self.fail_send = False

    def wave_tx_stop(self):
        self.busy = 0
        return 0

    def wave_tx_busy(self):
        return self.busy

    def wave_tx_at(self):
        return self.wave_id

    def set_mode(self, *args):
        return 0

    def write(self, *args):
        self.writes.append(args)
        return 0

    def wave_add_new(self):
        return 0

    def wave_add_generic(self, waveform):
        self.waveform = waveform
        return len(waveform)

    def wave_create(self):
        return self.wave_id

    def wave_send_once(self, wave_id):
        self.busy = 1
        return -1 if self.fail_send else 100

    def wave_delete(self, wave_id):
        self.deleted.append(wave_id)
        return 0

    def stop(self):
        self.connected = False


def config():
    return PigpioConfig(
        5,
        6,
        2,
        2,
        5,
        5,
        0.1,
        1000,
        1,
        verified=True,
        exclusive_wave_owner=True,
        waveform_hil_verified=True,
    )


@pytest.fixture
def engine():
    client = Client()
    adapter = PigpioPulseEngine(
        config(),
        client_factory=lambda *_: client,
        pulse_factory=lambda on, off, delay: SimpleNamespace(gpio_on=on, gpio_off=off, delay=delay),
    )
    yield adapter, client
    adapter.close()


def segment(adapter, count=20, identifier="s1", generation=1):
    return PulseSegment(identifier, count, 0.02, generation, adapter.identity.device_epoch, "cmd")


def test_finite_wave_timing_and_confirm_only_after_observed_completion(engine):
    adapter, client = engine
    adapter.open()
    request = segment(adapter)
    adapter.submit(request)
    assert adapter.poll() == []
    assert sum(p.delay for p in client.waveform) == 20000
    assert sum(bool(p.gpio_on & (1 << 5)) for p in client.waveform) == 20
    assert all(p.delay >= 2 for p in client.waveform)
    client.busy = 0
    (completion,) = adapter.poll()
    assert completion.signed_pulses == 20 and completion.command_id == "cmd"
    assert adapter.poll() == []
    assert client.deleted == [7]


def test_abort_never_invents_partial_count(engine):
    adapter, client = engine
    adapter.open()
    adapter.submit(segment(adapter, -20))
    result = adapter.abort()
    assert result.idle and result.confirmed_pulses is None
    assert adapter.poll() == []
    assert adapter.abort().confirmed_pulses == 0


def test_duplicate_submission_no_repeat_and_conflict_fails(engine):
    adapter, client = engine
    adapter.open()
    request = segment(adapter)
    adapter.submit(request)
    adapter.submit(request)
    with pytest.raises(DeviceError):
        adapter.submit(replace(request, signed_pulses=21))
    client.busy = 0
    adapter.poll()
    adapter.submit(request)
    assert client.busy == 0


def test_wrong_epoch_old_generation_busy_and_timing_rejected(engine):
    adapter, client = engine
    adapter.open()
    request = segment(adapter)
    with pytest.raises(DeviceError):
        adapter.submit(replace(request, device_epoch="old"))
    with pytest.raises(DeviceError):
        adapter.submit(replace(request, duration_s=0.00001))
    adapter.submit(request)
    with pytest.raises(DeviceError):
        adapter.submit(segment(adapter, identifier="s2"))
    client.busy = 0
    adapter.poll()
    with pytest.raises(DeviceError):
        adapter.submit(segment(adapter, identifier="s3", generation=0))


def test_replaced_wave_loses_completion_evidence(engine):
    adapter, client = engine
    adapter.open()
    adapter.submit(segment(adapter))
    client.wave_id = 999
    with pytest.raises(DeviceError) as caught:
        adapter.poll()
    assert caught.value.code == "PULSE_OWNERSHIP_LOST"


def test_process_ownership_lock_prevents_second_client(engine):
    adapter, client = engine
    adapter.open()
    second = PigpioPulseEngine(
        config(),
        client_factory=lambda *_: pytest.fail("must not connect"),
        pulse_factory=lambda *_: None,
    )
    with pytest.raises(DeviceError) as caught:
        second.open()
    assert caught.value.code == "DEVICE_BUSY"


def test_unverified_backend_refuses_gpio_and_connection():
    adapter = PigpioPulseEngine(
        replace(config(), waveform_hil_verified=False),
        client_factory=lambda *_: pytest.fail("must not connect"),
    )
    with pytest.raises(DeviceError):
        adapter.open()


def test_send_error_never_settles_as_success(engine):
    adapter, client = engine
    adapter.open()
    client.fail_send = True
    with pytest.raises(DeviceError):
        adapter.submit(segment(adapter))
    client.busy = 0
    with pytest.raises(DeviceError) as caught:
        adapter.poll()
    assert caught.value.code == "ROTATION_ESTIMATE_INVALID"
    assert adapter.abort().confirmed_pulses is None


def test_short_unobserved_wave_cannot_invent_completion(engine):
    adapter, client = engine
    adapter.open()
    client.wave_send_once = lambda _: 100  # never observed running
    adapter.submit(segment(adapter))
    with pytest.raises(DeviceError):
        adapter.poll()
    assert adapter.abort().confirmed_pulses is None


def test_preparation_delay_expires_permit_before_send_and_deletes_unused_wave(engine):
    adapter, client = engine
    now = [1_000_000_000]
    adapter.clock = lambda: now[0]
    adapter.open()
    request = replace(segment(adapter), expires_ns=now[0] + 60_000_000)
    sends = []

    def delayed_create():
        now[0] += 50_000_000
        return client.wave_id

    def forbidden_send(wave_id):
        sends.append(wave_id)
        pytest.fail("expired preparation must never trigger wave_send_once")

    client.wave_create = delayed_create
    client.wave_send_once = forbidden_send
    with pytest.raises(DeviceError) as caught:
        adapter.submit(request)
    assert caught.value.code == "PULSE_EXPIRED"
    assert sends == [] and client.deleted == [client.wave_id]
    assert adapter._active is None and adapter._wave_id is None
    assert adapter.is_idle() and adapter.poll() == []


def test_finite_permit_valid_at_final_trigger_is_accepted(engine):
    adapter, client = engine
    now = [1_000_000_000]
    adapter.clock = lambda: now[0]
    adapter.open()
    request = replace(segment(adapter), expires_ns=now[0] + 60_000_000)
    adapter.submit(request)
    assert client.busy == 1


@pytest.mark.parametrize("expiry", [True, -1, 1.5, float("nan")])
def test_invalid_permit_deadline_never_prepares_wave(engine, expiry):
    adapter, client = engine
    adapter.open()
    request = replace(segment(adapter), expires_ns=expiry)
    with pytest.raises(DeviceError) as caught:
        adapter.submit(request)
    assert caught.value.code == "INVALID_ARGUMENT"
    assert client.waveform == [] and not client.busy
