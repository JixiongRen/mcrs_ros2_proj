"""T09/T10/T11/T16/T17/T18 via real ROS processes and installed packages.

No driver mocks or direct calls into node callbacks are used here. Each test
owns its DDS domain and launch process group. Ordinary adapters run with an
independent simulated plant in the device node process.
"""

import math
import os
import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

import pytest
import rclpy
from gca_ros.cli import GcaClient
from gca_system_interfaces.action import ExecuteMotion
from gca_system_interfaces.srv import ControlSession, ManageDevice
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

pytestmark = pytest.mark.integration


@dataclass
class Rig:
    client: GcaClient
    launch: subprocess.Popen


@pytest.fixture
def rig(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", str(100 + os.getpid() % 100))
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    environment = dict(os.environ, ROS_LOG_DIR=str(tmp_path / "ros_logs"))
    with (tmp_path / "launch.log").open("w") as stream:
        launch = subprocess.Popen(
            ["ros2", "launch", "system_bringup", "simulation.launch.py"],
            cwd=tmp_path,
            env=environment,
            stdout=stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        rclpy.init(args=[])
        client = GcaClient(timeout_s=10)
        try:
            client.configure()
            client.activate()
            client.wait_state(lambda s: s.state == "DISARMED" and s.linear_velocity_valid, 10)
            yield Rig(client, launch)
        finally:
            try:
                client.close()
            finally:
                rclpy.shutdown()
                if launch.poll() is None:
                    os.killpg(launch.pid, signal.SIGINT)
                    try:
                        launch.wait(8)
                    except subprocess.TimeoutExpired:
                        os.killpg(launch.pid, signal.SIGKILL)
                        launch.wait(2)
                if launch.returncode != 0:
                    print((tmp_path / "launch.log").read_text()[-12000:])


def prepare(client, mode="AUTO", auto_renew=True, lease_s=4):
    client.claim(mode, lease_s=lease_s, auto_renew=auto_renew)
    client.manage("set_reference")
    client.wait_state(lambda s: s.linear_reference_valid and s.rotation_reference_valid)
    client.manage("arm")
    client.wait_state(lambda s: s.armed)


def test_active_is_not_arm_and_direct_device_token_cannot_be_source_token(rig):
    c = rig.client
    assert not c.state.armed
    c.claim()
    c.manage("set_reference")
    c.wait_state(lambda s: s.linear_reference_valid)
    assert not c.send_goal(c.make_goal()).accepted
    direct = ActionClient(
        c.node, ExecuteMotion, "/gca/execute_motion", callback_group=ReentrantCallbackGroup()
    )
    assert direct.wait_for_server(timeout_sec=5)
    assert not c.wait(direct.send_goal_async(c.make_goal())).accepted
    response = c.call(
        ManageDevice,
        "/gca/arm",
        ManageDevice.Request(context=c.context(), token=c.credentials.token),
    )
    assert not response.accepted
    direct.destroy()


def test_source_conflict_and_atomic_two_axis_rejection(rig):
    c = rig.client
    prepare(c)
    conflict = c.call(
        ControlSession,
        "/system/claim_control",
        ControlSession.Request(owner="second-client", mode="AUTO", lease_s=4.0),
    )
    assert not conflict.accepted
    start = c.state.linear_position_m
    invalid = c.make_goal(linear_mm=3, rotation_deg=1e6)
    handle = c.send_goal(invalid)
    if handle.accepted:
        result = c.wait(handle.get_result_async(), 5).result
        assert result.outcome == 3
    c.wait_state(lambda s: s.state in ("READY", "FAULT"))
    assert abs(c.state.linear_position_m - start) < 0.00005


def test_single_axis_negative_target_and_relative_roundtrip(rig):
    c = rig.client
    prepare(c)
    first = c.move(linear_mm=-1, rotation_deg=0, axis_mask=1)
    assert first.outcome == 1 and first.stop_confirmed
    second = c.move(linear_mm=1, rotation_deg=0, axis_mask=1, relative=True)
    assert second.outcome == 1
    assert abs(second.final_state.linear_position_m) < 0.00015
    rotation = c.move(linear_mm=0, rotation_deg=-3, axis_mask=2)
    assert rotation.outcome == 1
    assert abs(math.degrees(rotation.final_state.rotation_estimated_rad) + 3) < 0.04


def test_competing_goals_and_immediate_cancel_have_one_terminal(rig):
    c = rig.client
    prepare(c)
    goal = c.make_goal(linear_mm=30, rotation_deg=90, max_linear_mm_s=2)
    first = c.send_goal(goal)
    assert first.accepted
    other = c.send_goal(c.make_goal(linear_mm=-20))
    assert not other.accepted
    canceled = c.wait(first.cancel_goal_async())
    assert canceled.goals_canceling
    result_future = first.get_result_async()
    result = c.wait(result_future, 6).result
    assert result.outcome == 2 and result.stop_confirmed
    repeated = c.wait(first.get_result_async(), 2).result
    assert repeated.outcome == result.outcome
    assert repeated.final_state.active_command_id == result.final_state.active_command_id
    c.wait_state(lambda s: s.stop.progress == "CONFIRMED")


def test_lifecycle_deactivate_stops_active_motion_without_ros_wait_deadlock(rig):
    c = rig.client
    prepare(c)
    handle = c.send_goal(c.make_goal(linear_mm=40, max_linear_mm_s=2))
    assert handle.accepted
    c.wait_state(lambda s: abs(s.linear_velocity_m_s) > 0.0005)
    started = time.monotonic()
    c.lifecycle(4)  # TRANSITION_DEACTIVATE; local I/O keeps running during callback.
    assert time.monotonic() - started < 3
    state = c.wait_state(lambda s: not s.armed and s.stop.progress == "CONFIRMED")
    assert not state.lifecycle_active
    result = c.wait(handle.get_result_async(), 5).result
    assert result.stop_confirmed and result.outcome != 1
    assert not c.send_goal(c.make_goal()).accepted


def test_hold_source_expiry_stops_and_does_not_auto_rearm(rig):
    c = rig.client
    prepare(c)
    result = c.move(linear_mm=1, rotation_deg=2, hold=True)
    assert result.outcome == 1 and result.outputs_held
    c.wait_state(lambda s: s.state == "HOLDING")
    c._renew_stop.set()
    c._renew_thread.join(2)
    state = c.wait_state(lambda s: not s.armed and s.stop.progress == "CONFIRMED", 6)
    assert state.state in ("DISARMED", "FAULT")
    assert not c.send_goal(c.make_goal()).accepted


def test_control_freeze_does_not_prevent_independent_io_stop(rig):
    c = rig.client
    prepare(c)
    handle = c.send_goal(c.make_goal(linear_mm=30, max_linear_mm_s=2))
    assert handle.accepted
    c.wait_state(lambda s: s.linear_velocity_m_s > 0.0005)
    c.fault("control_freeze")
    # DDS publishes stale state, with advancing raw sample age; it cannot renew output.
    c.wait_state(lambda s: s.feedback_age_s > 0.25, 2)
    c.fault("control_freeze", False)
    state = c.wait_state(lambda s: s.state == "FAULT" and s.stop.progress == "CONFIRMED", 5)
    assert state.fault_code
    result = c.wait(handle.get_result_async(), 5).result
    assert result.outcome == 3 and result.stop_confirmed


def test_simulation_configuration_resource_is_installed():
    from ament_index_python.packages import get_package_share_directory

    share = Path(get_package_share_directory("system_bringup"))
    assert (share / "config/simulation.yaml").is_file()
    assert (share / "launch/simulation.launch.py").is_file()


def test_cancel_wins_over_deferred_hold_candidate(rig):
    c = rig.client
    prepare(c)
    c.fault("terminal_pause")
    handle = c.send_goal(c.make_goal(linear_mm=1, rotation_deg=2, hold=True))
    assert handle.accepted
    future = handle.get_result_async()
    c.wait_state(lambda s: s.state == "HOLDING", 8)
    assert not future.done()
    assert c.wait(handle.cancel_goal_async()).goals_canceling
    c.fault("terminal_pause", False)
    result = c.wait(future, 6).result
    assert result.outcome == 2 and result.stop_confirmed and not result.outputs_held
    c.wait_state(lambda s: s.stop.progress == "CONFIRMED" and s.state != "HOLDING")


def test_committed_success_cannot_be_canceled_and_slow_telemetry_does_not_block_control(rig):
    c = rig.client
    prepare(c)
    c.fault("telemetry_pause")
    try:
        handle = c.send_goal(c.make_goal(linear_mm=1, rotation_deg=2))
        assert handle.accepted
        result = c.wait(handle.get_result_async(), 8).result
        assert result.outcome == 1 and result.stop_confirmed
        assert abs(result.final_state.linear_position_m - 0.001) < 0.00012
        assert not c.wait(handle.cancel_goal_async()).goals_canceling
    finally:
        # A failed assertion must not leave this test's observation stall
        # active while the fixture requests and confirms its bounded stop.
        c.fault("telemetry_pause", False)


def test_delayed_goal_permit_expires_while_source_lease_keeps_renewing(rig):
    c = rig.client
    prepare(c)
    goal = c.make_goal(linear_mm=1, rotation_deg=2)
    before = c.state.linear_position_m
    time.sleep(goal.admission_timeout_s + 0.2)
    assert c.renewal_error is None and c.state.armed
    assert not c.send_goal(goal).accepted
    assert abs(c.state.linear_position_m - before) < 0.00005


def test_configuration_changes_are_transactional_and_structural_change_is_rejected(rig):
    import yaml
    from gca.config import RuntimeConfig
    from gca_system_interfaces.srv import ApplyConfig

    c = rig.client
    c.claim()
    values = RuntimeConfig().as_dict()
    values.update(revision="integration-gains-v2", kp=2.5)
    c.apply_config(yaml.safe_dump(values))
    state = c.wait_state(lambda s: s.applied_config_revision == "integration-gains-v2")
    assert state.requested_config_revision == state.applied_config_revision
    values.update(revision="integration-invalid-queue", queue_capacity=999)
    rejected = c.call(
        ApplyConfig,
        "/system/apply_config",
        ApplyConfig.Request(
            context=c.context(),
            token=c.credentials.token,
            configuration_yaml=yaml.safe_dump(values),
        ),
    )
    assert not rejected.accepted
    assert (
        "RECONFIGURE_REQUIRED" in rejected.error_code or "RECONFIGURE_REQUIRED" in rejected.message
    )
    assert c.state.applied_config_revision == "integration-gains-v2"


@pytest.mark.parametrize("shutdown_signal", [signal.SIGINT, signal.SIGTERM])
def test_cli_signal_during_jog_stops_and_releases_owned_session(rig, tmp_path, shutdown_signal):
    """Kill only this test's input process; its old Jog must not keep moving."""
    c = rig.client
    with (tmp_path / "shell.log").open("w") as stream:
        process = subprocess.Popen(
            ["ros2", "run", "gca_ros", "gca_cli", "shell"],
            stdin=subprocess.PIPE,
            stdout=stream,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        try:
            process.stdin.write("configure\nactivate\nclaim MANUAL\nreference\narm\njog 2 10 20\n")
            process.stdin.flush()
            c.wait_state(lambda s: s.armed and s.linear_velocity_m_s > 0.0005, 10)
            os.killpg(process.pid, shutdown_signal)
            process.wait(10)
            state = c.wait_state(lambda s: not s.armed and s.stop.progress == "CONFIRMED", 6)
            assert abs(state.linear_velocity_m_s) <= 0.00015
        finally:
            process.stdin.close()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(2)
