"""Make an unexpected child exit fail the owning launch and stop its other node."""

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


def fail_on_unexpected_exit():
    def exited(event, context):
        if not context.is_shutdown:
            raise RuntimeError(f"GCA launch child exited unexpectedly (code {event.returncode})")
        return None

    return RegisterEventHandler(OnProcessExit(on_exit=exited))
