from __future__ import annotations

import pytest

from wpilib_agent_tools.cli import build_parser
from wpilib_agent_tools.commands import graph
from wpilib_agent_tools.commands.graph import _expand_keys, _numeric_points


def test_numeric_points_reports_non_numeric_samples() -> None:
    points = [
        (0.0, 1.0),
        (1.0, {"type": "ChassisSpeeds", "vx_mps": 2.0}),
        (2.0, [3.0, 4.0]),
        (3.0, "not-graphable"),
        (4.0, True),
    ]
    numeric, skipped_non_numeric = _numeric_points(points)
    assert numeric == [(0.0, 1.0), (2.0, 3.0), (4.0, 1.0)]
    assert skipped_non_numeric == 2


def test_expand_keys_accepts_repeated_and_comma_separated_values() -> None:
    keys = _expand_keys(["Drive/Vx, Drive/Vy", "Shooter/RPM", "Arm/Pos,"])
    assert keys == ["Drive/Vx", "Drive/Vy", "Shooter/RPM", "Arm/Pos"]


def test_load_pyplot_surfaces_dependency_error(monkeypatch: pytest.MonkeyPatch) -> None:
    def fake_import_module(_name: str) -> None:
        raise ModuleNotFoundError("matplotlib missing")

    monkeypatch.setattr(graph.importlib, "import_module", fake_import_module)

    with pytest.raises(RuntimeError, match="matplotlib is required for graph command"):
        graph._load_pyplot()


def test_build_parser_does_not_load_pyplot(monkeypatch: pytest.MonkeyPatch) -> None:
    def fail_if_called() -> None:
        raise AssertionError("_load_pyplot should not run during parser construction")

    monkeypatch.setattr(graph, "_load_pyplot", fail_if_called)
    parser = build_parser()
    args = parser.parse_args(["logs"])
    assert args.command == "logs"
