"""Independent golden frames and adversarial encoder stream tests (T03)."""

import pytest
from gca.protocols.encoder import (
    CountFormat,
    DeviceException,
    ProtocolError,
    ResponseParser,
    append_crc,
    crc16,
    parse_response,
    read_request,
)


def test_crc_golden_width_and_standard_vector():
    assert crc16(b"123456789") == 0x4B37
    assert read_request(1, 0x24, 2) == bytes.fromhex("01 03 00 24 00 02 84 00")
    assert read_request(1, 0, 2) == bytes.fromhex("01 03 00 00 00 02 c4 0b")


def test_fragmented_and_two_concatenated_responses():
    frame = append_crc(bytes.fromhex("01 03 04 00 00 00 2a"))
    parser = ResponseParser(1, 4)
    assert parser.feed(frame[:1]) == ()
    assert parser.feed(frame[1:6]) == ()
    assert parser.feed(frame[6:] + frame) == (bytes.fromhex("00 00 00 2a"),) * 2
    assert not parser.buffer


@pytest.mark.parametrize(
    "frame",
    [
        b"",
        b"\x01\x03",
        bytes.fromhex("01 03 04 00 00 00 00 00 00"),
        append_crc(bytes.fromhex("02 03 04 00 00 00 00")),
        append_crc(bytes.fromhex("01 04 04 00 00 00 00")),
        append_crc(bytes.fromhex("01 03 02 00 00")),
        append_crc(bytes.fromhex("01 03 04 00 00 00 00 00")),
    ],
)
def test_malformed_never_becomes_measurement(frame):
    with pytest.raises(ProtocolError):
        parse_response(frame, address=1, data_bytes=4)


def test_exception_and_buffer_limit_are_explicit():
    with pytest.raises(DeviceException) as caught:
        ResponseParser(1, 4).feed(append_crc(bytes.fromhex("01 83 02")))
    assert caught.value.exception_code == 2
    with pytest.raises(ProtocolError, match="limit"):
        ResponseParser(1, 4).feed(b"x" * 261)


def test_signed_count_and_no_invented_wrap_rule():
    assert CountFormat(32, True).decode(bytes.fromhex("ff ff ff ff")) == -1
    assert CountFormat(32, False).decode(bytes.fromhex("ff ff ff ff")) == 4294967295
    with pytest.raises(ProtocolError):
        CountFormat(32, False, modulus=4096).decode(bytes.fromhex("00 00 10 00"))
    with pytest.raises(ProtocolError):
        CountFormat(32, False).decode(b"\x00")


@pytest.mark.parametrize("args", [(0, 0, 2), (1, -1, 2), (1, 65535, 2), (1, 0, 0)])
def test_invalid_request_rejected(args):
    with pytest.raises(ValueError):
        read_request(*args)
