"""Strict Modbus-RTU function-03 framing for a configured encoder.

The old 01/03/06 frames are evidence for a candidate protocol, not proof of
an encoder model. Address, register layout and count interpretation must be
confirmed before hardware use. Only one transaction may be outstanding.
"""

from __future__ import annotations

from dataclasses import dataclass


class ProtocolError(ValueError):
    """A complete response cannot be used as a measurement."""


class DeviceException(ProtocolError):
    """A valid Modbus exception response, distinct from a corrupt frame."""

    def __init__(self, exception_code: int):
        self.exception_code = exception_code
        super().__init__(f"encoder exception 0x{exception_code:02x}")


def crc16(data: bytes) -> int:
    """Return Modbus CRC16; serialized CRC is always two little-endian bytes."""
    value = 0xFFFF
    for octet in data:
        value ^= octet
        for _ in range(8):
            value = (value >> 1) ^ (0xA001 if value & 1 else 0)
    return value


def append_crc(data: bytes) -> bytes:
    return data + crc16(data).to_bytes(2, "little")


def read_request(address: int, register: int, registers: int) -> bytes:
    """Encode a read request. No maintenance/write commands are implicit."""
    if not 1 <= address <= 247 or not 0 <= register <= 65535:
        raise ValueError("invalid address or register")
    if not 1 <= registers <= 125 or register + registers > 65536:
        raise ValueError("invalid register range")
    return append_crc(
        bytes((address, 3)) + register.to_bytes(2, "big") + registers.to_bytes(2, "big")
    )


def parse_response(frame: bytes, *, address: int, data_bytes: int) -> bytes:
    """Validate address, function, exact length and CRC before exposing data."""
    if len(frame) < 5:
        raise ProtocolError("short encoder response")
    if frame[0] != address:
        raise ProtocolError("unexpected encoder address")
    if crc16(frame[:-2]).to_bytes(2, "little") != frame[-2:]:
        raise ProtocolError("encoder CRC mismatch")
    if frame[1] == 0x83:
        if len(frame) != 5:
            raise ProtocolError("invalid exception length")
        raise DeviceException(frame[2])
    if frame[1] != 3:
        raise ProtocolError("unexpected encoder function")
    if frame[2] != data_bytes or len(frame) != data_bytes + 5:
        raise ProtocolError("encoder response length mismatch")
    return frame[3:-2]


@dataclass(frozen=True)
class CountFormat:
    """Explicit count format. modulo is configured only from a device manual."""

    bits: int
    signed: bool
    byteorder: str = "big"
    modulus: int | None = None

    def __post_init__(self) -> None:
        if self.bits not in (16, 32, 64) or self.byteorder not in ("big", "little"):
            raise ValueError("unsupported count format")
        if self.modulus is not None and not 1 < self.modulus <= 2**self.bits:
            raise ValueError("invalid count modulus")

    def decode(self, payload: bytes) -> int:
        if len(payload) * 8 != self.bits:
            raise ProtocolError("count payload width mismatch")
        value = int.from_bytes(payload, self.byteorder, signed=self.signed)
        if self.modulus is not None and not 0 <= value < self.modulus:
            raise ProtocolError("raw count outside configured modulo range")
        return value


class ResponseParser:
    """Bounded incremental parser; malformed input fails rather than resyncing.

    ``feed`` returns complete payloads, allowing the transaction owner to reject
    multiple responses. A new parser is created for every request; it must not
    make a late reply valid simply by assigning it a new sample timestamp.
    """

    def __init__(self, address: int, data_bytes: int, max_buffer: int = 260):
        if not 1 <= address <= 247 or not 1 <= data_bytes <= 250:
            raise ValueError("invalid response contract")
        if max_buffer < data_bytes + 5 or max_buffer > 4096:
            raise ValueError("invalid parser bound")
        self.address, self.data_bytes = address, data_bytes
        self.max_buffer = max_buffer
        self.buffer = bytearray()

    def feed(self, data: bytes) -> tuple[bytes, ...]:
        if len(self.buffer) + len(data) > self.max_buffer:
            self.buffer.clear()
            raise ProtocolError("encoder buffer limit exceeded")
        self.buffer.extend(data)
        result = []
        while len(self.buffer) >= 2:
            if self.buffer[0] != self.address:
                raise ProtocolError("unexpected encoder address")
            function = self.buffer[1]
            if function not in (3, 0x83):
                raise ProtocolError("unexpected encoder function")
            if len(self.buffer) < 3:
                break
            if function == 3 and self.buffer[2] != self.data_bytes:
                raise ProtocolError("unexpected encoder byte count")
            expected = 5 if function == 0x83 else self.data_bytes + 5
            if len(self.buffer) < expected:
                break
            frame = bytes(self.buffer[:expected])
            del self.buffer[:expected]
            result.append(parse_response(frame, address=self.address, data_bytes=self.data_bytes))
        return tuple(result)
