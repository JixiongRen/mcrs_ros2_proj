"""Deterministically enqueue in the empty-check/finally worker-exit window."""

import threading
import time

import rclpy
from gca_system_interfaces.msg import CommandContext, JogCommand
from rclpy.executors import MultiThreadedExecutor
from system_coordinator.node import SystemCoordinator


def test_jog_worker_exit_and_new_ingress_schedule_exactly_one_successor():
    rclpy.init(args=["--ros-args", "-p", "coordinator_key:=test-jog-worker-capability"])
    node = SystemCoordinator()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    reached_finally, allow_finally = threading.Event(), threading.Event()
    created, errors = [], []
    original_create = executor.create_task
    original_finish = node._finish_jog_worker

    def tracked_create(*args, **kwargs):
        task = original_create(*args, **kwargs)
        created.append(task)
        return task

    def paused_finish():
        if not reached_finally.is_set():
            reached_finally.set()
            if not allow_finally.wait(2.0):
                raise AssertionError("controlled finally barrier timed out")
        original_finish()

    def drain_empty_worker():
        coroutine = node._forward_jog()
        try:
            coroutine.send(None)
        except StopIteration:
            return
        except BaseException as exc:
            errors.append(exc)
        finally:
            coroutine.close()

    executor.create_task = tracked_create
    node._finish_jog_worker = paused_finish
    node._jog_forwarding = True
    thread = threading.Thread(target=drain_empty_worker)
    try:
        source = node._authority.claim("operator", "MANUAL", 2.0)
        context = CommandContext(
            session_id=source.session_id,
            generation=source.generation,
            sequence=2,
            task_id="task",
            command_id="jog",
        )
        envelope, _ = node._motion_permits.issue(source.token, context, 0.5, source.deadline)
        thread.start()
        assert reached_finally.wait(1.0)
        # Ownership is retained until the atomic finally/handoff operation.
        assert node._jog_forwarding
        node._jog(
            JogCommand(context=context, token=envelope, deadman=True, axis_mask=3, ttl_s=0.15)
        )
        assert node._pending_jog is not None
        assert created == []
        allow_finally.set()
        thread.join(1.0)
        assert not thread.is_alive() and errors == []
        assert len(created) == 1
        assert node._jog_forwarding
        # Finish the owned successor without contacting any external device.
        with node._lock:
            node._pending_jog = None
        deadline = time.monotonic() + 1.0
        while not created[0].done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert created[0].done()
        assert not node._jog_forwarding
    finally:
        allow_finally.set()
        if thread.is_alive():
            thread.join(2.0)
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        rclpy.shutdown()
