from __future__ import annotations

import struct

from wpilib_agent_tools.lib.log_reader import decode_value


def test_decode_value_uses_struct_decoder() -> None:
    raw = struct.pack("<ddd", 0.5, 1.25, -0.8)
    decoded = decode_value(raw, "struct:ChassisSpeeds")
    assert decoded["type"] == "ChassisSpeeds"
    assert decoded["vx_mps"] == 0.5
    assert decoded["vy_mps"] == 1.25
    assert decoded["omega_radps"] == -0.8


def test_decode_value_unknown_struct_returns_human_readable_fallback() -> None:
    decoded = decode_value(b"\x01\x02\xab", "struct:MyUnknownStruct")
    assert decoded["type"] == "UnknownStruct"
    assert decoded["wpilog_type"] == "struct:MyUnknownStruct"
    assert decoded["raw_size_bytes"] == 3
    assert decoded["raw_hex"] == "0x0102ab"
