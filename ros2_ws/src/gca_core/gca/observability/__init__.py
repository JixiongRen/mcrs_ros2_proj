"""Optional bounded observation sinks; never called synchronously by device I/O."""

from .recorder import AsyncRecorder

__all__ = ["AsyncRecorder"]
