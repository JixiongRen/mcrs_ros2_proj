"""The telemetry facade must not cause plant integration or a device operation."""

from gca.runtime import ControlRuntime


def test_simulation_evidence_is_read_only_and_reports_actual_accepted_output():
    from gca.simulation.devices import simulated_devices

    runtime = ControlRuntime()
    runtime.servo, runtime.encoder, runtime.pulse, runtime.plant = simulated_devices(runtime.config)
    runtime.plant.accept_velocity(1.25, runtime.clock())
    before = runtime.plant.snapshot()
    updated = runtime.plant._now
    runtime.plant.advance = lambda _: (_ for _ in ()).throw(AssertionError("advance"))
    runtime.pulse.poll = lambda: (_ for _ in ()).throw(AssertionError("poll"))
    evidence = runtime.simulation_evidence()
    assert evidence["available"]
    assert evidence["evidence_kind"] == "SIMULATED_ACCEPTED_IO_AND_PLANT"
    assert evidence["accepted_velocity_mm_s"] == 1.25
    assert evidence["plant_updated_ns"] == updated
    assert runtime.plant.snapshot() == before


def test_simulation_evidence_never_reads_hardware_ports():
    runtime = ControlRuntime()
    object.__setattr__(runtime.config, "backend", "hardware")
    assert runtime.simulation_evidence() == {"available": False, "evidence_kind": "SIMULATION_ONLY"}
