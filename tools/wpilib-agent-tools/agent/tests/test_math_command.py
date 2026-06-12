from __future__ import annotations

import argparse

from wpilib_agent_tools.cli import build_parser
from wpilib_agent_tools.commands.math import _run_math_operation


def _args(**overrides: object) -> argparse.Namespace:
    base: dict[str, object] = {
        "mode": "deriv",
        "expr": None,
        "equation": None,
        "var": "x",
        "value": None,
        "json": True,
    }
    base.update(overrides)
    return argparse.Namespace(**base)


def test_math_modes_deriv_integral_simplify_solve_eval() -> None:
    deriv = _run_math_operation(_args(mode="deriv", expr="x**3"))
    assert deriv["result"] == "3*x**2"

    integral = _run_math_operation(_args(mode="integral", expr="2*x"))
    assert integral["result"] == "x**2"

    simplified = _run_math_operation(_args(mode="simplify", expr="sin(x)**2 + cos(x)**2"))
    assert simplified["result"] == "1"

    solved = _run_math_operation(_args(mode="solve", equation="x**2-4=0"))
    assert set(solved["solutions"]) == {"-2", "2"}

    evaluated = _run_math_operation(
        _args(mode="eval", expr="x**2 + y", value=["x=3", "y=2"])
    )
    assert evaluated["result"] == 11.0


def test_math_cli_parser_registers_math_command() -> None:
    parser = build_parser()
    args = parser.parse_args(["math", "--mode", "deriv", "--expr", "x**2"])
    assert args.command == "math"
    assert args.mode == "deriv"
