from __future__ import annotations

from wpilib_agent_tools.commands.query import _json_safe


def test_json_safe_converts_bytes_recursively() -> None:
    payload = {
        "results": [
            {
                "values": [(1.0, b"\x01\x02"), (2.0, {"raw": b"\xff"})],
            }
        ]
    }
    safe = _json_safe(payload)
    values = safe["results"][0]["values"]
    assert values[0][1] == "0x0102"
    assert values[1][1]["raw"] == "0xff"


def test_json_safe_keeps_struct_dict_human_readable() -> None:
    payload = {
        "results": [
            {
                "values": [
                    (
                        1.0,
                        {
                            "type": "ChassisSpeeds",
                            "vx_mps": 0.2,
                            "vy_mps": 0.0,
                            "omega_radps": -0.1,
                        },
                    )
                ]
            }
        ]
    }
    safe = _json_safe(payload)
    value = safe["results"][0]["values"][0][1]
    assert value["type"] == "ChassisSpeeds"
    assert value["vx_mps"] == 0.2
