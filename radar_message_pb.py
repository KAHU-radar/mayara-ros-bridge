"""
Hand-coded protobuf decoder for RadarMessage.

Matches the proto3 schema:
    message RadarMessage {
        uint32 radar = 1;
        message Spoke {
            uint32 angle = 1;
            optional uint32 bearing = 2;
            uint32 range = 3;
            optional uint64 time = 4;
            bytes data = 5;
            optional int64 lat = 6;
            optional int64 lon = 7;
        }
        repeated Spoke spokes = 2;
    }

We decode the wire format directly — no protobuf library needed.
"""

import struct
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class Spoke:
    angle: int = 0
    bearing: Optional[int] = None
    range: int = 0
    time: Optional[int] = None
    data: bytes = b""
    lat: Optional[int] = None
    lon: Optional[int] = None


@dataclass
class RadarMessage:
    radar: int = 0
    spokes: list = field(default_factory=list)


def _decode_varint(buf: bytes, pos: int) -> tuple[int, int]:
    """Decode a varint from buf starting at pos. Returns (value, new_pos)."""
    result = 0
    shift = 0
    while pos < len(buf):
        b = buf[pos]
        pos += 1
        result |= (b & 0x7F) << shift
        if (b & 0x80) == 0:
            return result, pos
        shift += 7
    raise ValueError("Truncated varint")


def _decode_signed_varint(buf: bytes, pos: int) -> tuple[int, int]:
    """Decode a signed varint (zigzag for sint, or cast for int64)."""
    val, pos = _decode_varint(buf, pos)
    # proto3 int64 uses two's complement in varint encoding
    if val > 0x7FFFFFFFFFFFFFFF:
        val -= 0x10000000000000000
    return val, pos


def _decode_spoke(buf: bytes) -> Spoke:
    """Decode a Spoke sub-message from its length-delimited bytes."""
    spoke = Spoke()
    pos = 0
    end = len(buf)

    while pos < end:
        tag, pos = _decode_varint(buf, pos)
        field_number = tag >> 3
        wire_type = tag & 0x07

        if wire_type == 0:  # varint
            val, pos = _decode_varint(buf, pos)
            if field_number == 1:
                spoke.angle = val
            elif field_number == 2:
                spoke.bearing = val
            elif field_number == 3:
                spoke.range = val
            elif field_number == 4:
                spoke.time = val
            elif field_number == 6:
                # int64 - need signed interpretation
                if val > 0x7FFFFFFFFFFFFFFF:
                    val -= 0x10000000000000000
                spoke.lat = val
            elif field_number == 7:
                if val > 0x7FFFFFFFFFFFFFFF:
                    val -= 0x10000000000000000
                spoke.lon = val

        elif wire_type == 2:  # length-delimited
            length, pos = _decode_varint(buf, pos)
            data = buf[pos : pos + length]
            pos += length
            if field_number == 5:
                spoke.data = data

        elif wire_type == 1:  # 64-bit fixed
            pos += 8
        elif wire_type == 5:  # 32-bit fixed
            pos += 4
        else:
            raise ValueError(f"Unknown wire type {wire_type} at field {field_number}")

    return spoke


def decode_radar_message(buf: bytes) -> RadarMessage:
    """Decode a RadarMessage from protobuf bytes."""
    msg = RadarMessage()
    pos = 0
    end = len(buf)

    while pos < end:
        tag, pos = _decode_varint(buf, pos)
        field_number = tag >> 3
        wire_type = tag & 0x07

        if wire_type == 0:  # varint
            val, pos = _decode_varint(buf, pos)
            if field_number == 1:
                msg.radar = val

        elif wire_type == 2:  # length-delimited
            length, pos = _decode_varint(buf, pos)
            data = buf[pos : pos + length]
            pos += length
            if field_number == 2:
                spoke = _decode_spoke(data)
                msg.spokes.append(spoke)

        elif wire_type == 1:  # 64-bit fixed
            pos += 8
        elif wire_type == 5:  # 32-bit fixed
            pos += 4
        else:
            raise ValueError(f"Unknown wire type {wire_type}")

    return msg
