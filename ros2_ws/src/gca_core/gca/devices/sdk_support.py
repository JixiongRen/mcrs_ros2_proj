"""Audited glue for dynamixel-sdk 4.1.0 and pigpio 1.78.

The SDK handles wire CRC/stuffing. These small subclasses add a monotonic
transaction budget and validate response parameter length *before* the SDK
convenience methods can accidentally expose CRC octets as data.
"""

from __future__ import annotations

import socket
from collections.abc import Callable
from contextlib import nullcontext
from importlib.metadata import version

from .common import DeviceError


def dynamixel_factories(clock: Callable[[], int]):
    """Load the official SDK without opening a serial device."""
    try:
        import dynamixel_sdk as sdk
        from dynamixel_sdk.protocol2_packet_handler import Protocol2PacketHandler
    except ImportError as exc:
        raise DeviceError("DEPENDENCY_MISSING", "install dynamixel-sdk==4.1.0") from exc

    if version("dynamixel-sdk") != "4.1.0":
        raise DeviceError("DEPENDENCY_VERSION", "expected dynamixel-sdk==4.1.0")

    class DeadlinePort(sdk.PortHandler):
        deadline_ns = 0
        write_guard = None

        def setupPort(self, cflag_baud):
            """Pinned SDK setup with POSIX exclusivity requested before open."""
            import serial

            if self.is_open:
                self.closePort()
            self.ser = serial.Serial(
                port=self.port_name,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                timeout=0,
                exclusive=True,
            )
            self.is_open = True
            self.ser.reset_input_buffer()
            self.tx_time_per_byte = (1000.0 / self.baudrate) * 10.0
            return True

        def getCurrentTime(self):
            return clock() / 1_000_000

        def isPacketTimeout(self):
            # The caller already supplies the complete monotonic I/O budget.
            # SDK latency estimates must not silently shorten that deadline.
            return clock() >= self.deadline_ns

        def readPort(self, length):
            remaining = (self.deadline_ns - clock()) / 1e9
            if remaining <= 0:
                return b""
            # Wait for bytes in the serial driver instead of spinning the
            # SDK's nonblocking receive loop alongside the control threads.
            self.ser.timeout = remaining
            return super().readPort(length)

        def writePort(self, packet):
            # Only final authorization and transmission share the STOP fence;
            # the SDK's subsequent receive wait must not block new inputs.
            with (self.write_guard or nullcontext)():
                remaining = (self.deadline_ns - clock()) / 1e9
                if remaining <= 0:
                    raise DeviceError("DEVICE_TIMEOUT", "DYNAMIXEL write expired")
                self.ser.write_timeout = remaining
                return super().writePort(packet)

    class CheckedPacketHandler(Protocol2PacketHandler):
        def removeStuffing(self, packet):
            # SDK rxPacket verifies the wire CRC before this normalization.
            # Version 4.1.0 moves the CRC and updates LENGTH but leaves stale
            # trailing list entries after removing stuffed bytes.
            normalized = super().removeStuffing(packet)
            length = normalized[5] | normalized[6] << 8
            del normalized[7 + length :]
            return normalized

        def txRxPacket(self, port, txpacket):
            device_id, instruction = txpacket[4], txpacket[7]
            if instruction == 2:
                expected = txpacket[10] | txpacket[11] << 8
            elif instruction == 1:
                expected = 3
            else:
                expected = 0
            try:
                received, result, error = super().txRxPacket(port, txpacket)
            except Exception:
                # Official SDK sets this before TX but has no exception cleanup.
                # A revoked write must not leave the following STOP bus-busy.
                port.is_using = False
                raise
            if result == sdk.COMM_SUCCESS:
                if (
                    received is None
                    or len(received) != 11 + expected
                    or received[4] != device_id
                    or received[7] != 0x55
                    or received[5] | received[6] << 8 != len(received) - 7
                ):
                    return received, sdk.COMM_RX_CORRUPT, error
            return received, result, error

    return DeadlinePort, CheckedPacketHandler


def bounded_pigpio_client(host: str, port: int, timeout_s: float):
    """Create the pinned client's wave-only connection with a socket deadline.

    pigpio.pi.__init__ creates an unbounded command socket and notification
    thread. Our wave-only adapter needs no notifications. We initialize the
    pinned implementation's socket holder directly, then use its public wave
    methods. This private initialization is version-pinned and tested; GPIO
    callbacks are deliberately unavailable on this client.
    """
    if host not in ("localhost", "127.0.0.1"):
        raise DeviceError(
            "CONFIG_INVALID", "pigpio backend is local IPv4 only; DNS hosts are unsupported"
        )
    host = "127.0.0.1"  # Normalize before connect: never call getaddrinfo/DNS.
    try:
        import pigpio
    except ImportError as exc:
        raise DeviceError("DEPENDENCY_MISSING", "install pigpio==1.78") from exc
    if version("pigpio") != "1.78":
        raise DeviceError("DEPENDENCY_VERSION", "expected pigpio==1.78")
    client = pigpio.pi.__new__(pigpio.pi)
    client.connected = False
    client._notify = None
    client._host, client._port = host, port
    client.sl = pigpio._socklock()
    connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        connection.settimeout(timeout_s)
        connection.connect((host, port))
        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except OSError:
        connection.close()
        raise
    client.sl.s = connection
    client.connected = True
    return client
