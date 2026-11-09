"""Compatibility entry point; implementation lives in the standalone client package."""

from gca_client.cli import interactive_shell, main, run_demo
from gca_client.client import Credentials, GcaClient, finite_float, save_credentials

__all__ = [
    "Credentials",
    "GcaClient",
    "finite_float",
    "save_credentials",
    "interactive_shell",
    "main",
    "run_demo",
]
