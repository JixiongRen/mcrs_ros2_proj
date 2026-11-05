"""Hardware assembly must be explicit and must never connect during construction."""

from dataclasses import asdict
from types import SimpleNamespace

import pytest
from gca.devices.common import DeviceError, DynamixelConfig, EncoderConfig, PigpioConfig
from gca.devices.factory import hardware_devices


def config():
    return SimpleNamespace(
        backend="hardware",
        hardware_verified=True,
        io_timeout_s=0.05,
        output_ttl_s=0.1,
        pulse_segment_s=0.03,
        max_linear_mm_s=1,
        encoder_count_modulus=0,
        hardware={
            "servo": asdict(
                DynamixelConfig("/no-servo", 57600, 1, 1020, 40, 0.229, 500, 3, verified=True)
            ),
            "encoder": asdict(
                EncoderConfig("/no-encoder", 38400, 1, 0, 32, True, 0.02, verified=True)
            ),
            "pulse": asdict(
                PigpioConfig(
                    5,
                    6,
                    2,
                    2,
                    5,
                    5,
                    0.05,
                    1000,
                    1,
                    verified=True,
                    exclusive_wave_owner=True,
                    waveform_hil_verified=True,
                )
            ),
        },
    )


def test_full_hardware_factory_constructs_all_real_adapters_without_io():
    servo, encoder, pulse = hardware_devices(config())
    assert servo.port is encoder.serial is pulse.client is None


@pytest.mark.parametrize(
    "device,field",
    [
        ("servo", "mm_per_revolution"),
        ("servo", "model_number"),
        ("encoder", "bits"),
        ("encoder", "quiet_s"),
        ("pulse", "direction_setup_us"),
        ("pulse", "waveform_hil_verified"),
    ],
)
def test_missing_physical_field_cannot_inherit_simulation(device, field):
    c = config()
    del c.hardware[device][field]
    with pytest.raises(DeviceError):
        hardware_devices(c)


def test_one_flag_cannot_enable_missing_hardware():
    c = config()
    c.hardware = {}
    with pytest.raises(DeviceError):
        hardware_devices(c)


def test_electrical_and_wave_verification_required():
    c = config()
    c.hardware["pulse"]["waveform_hil_verified"] = False
    with pytest.raises(DeviceError):
        hardware_devices(c)


def test_schema_rejects_boolean_gpio_and_string_number():
    c = config()
    c.hardware["pulse"]["pulse_gpio"] = True
    with pytest.raises(DeviceError):
        hardware_devices(c)
    c = config()
    c.hardware["servo"]["baudrate"] = "57600"
    with pytest.raises(DeviceError):
        hardware_devices(c)


def test_protocol_and_estimator_modulus_must_match():
    c = config()
    c.encoder_count_modulus = 4096
    with pytest.raises(DeviceError, match="modulus"):
        hardware_devices(c)
    c.hardware["encoder"]["modulus"] = 4096
    assert len(hardware_devices(c)) == 3
    c.encoder_count_modulus = 0
    with pytest.raises(DeviceError, match="modulus"):
        hardware_devices(c)


def test_remote_or_dns_pigpio_host_cannot_enable_backend():
    for host in ("host.example", "192.0.2.1", "::1"):
        c = config()
        c.hardware["pulse"]["host"] = host
        with pytest.raises(DeviceError, match="local"):
            hardware_devices(c)
