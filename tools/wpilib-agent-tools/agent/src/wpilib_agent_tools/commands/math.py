"""Symbolic and numeric calculator command."""

from __future__ import annotations

import argparse
import json
from typing import Any

from wpilib_agent_tools.lib.math_solver import (
    evaluate_expression,
    parse_assignments,
    solve_equation,
    symbolic_derivative,
    symbolic_integral,
    symbolic_simplify,
)


def register_subparser(subparsers: argparse._SubParsersAction) -> None:
    parser = subparsers.add_parser("math", help="Symbolic calculus and equation solving.")
    parser.add_argument(
        "--mode",
        required=True,
        choices=["deriv", "integral", "simplify", "solve", "eval"],
        help="Math operation mode.",
    )
    parser.add_argument("--expr", help="Expression for deriv/integral/simplify/eval modes.")
    parser.add_argument("--equation", help="Equation for solve mode. Example: x**2-4=0")
    parser.add_argument("--var", default="x", help="Variable symbol (default: x).")
    parser.add_argument(
        "--value",
        action="append",
        help="Variable assignment for eval mode in name=value form. Repeat as needed.",
    )
    parser.add_argument("--json", action="store_true", help="Emit JSON output.")
    parser.set_defaults(handler=handle_math)


def _run_math_operation(args: argparse.Namespace) -> dict[str, Any]:
    payload: dict[str, Any] = {"mode": args.mode, "var": args.var}

    if args.mode == "deriv":
        if not args.expr:
            raise ValueError("--expr is required for mode 'deriv'")
        payload["expr"] = args.expr
        payload["result"] = symbolic_derivative(args.expr, args.var)
        return payload

    if args.mode == "integral":
        if not args.expr:
            raise ValueError("--expr is required for mode 'integral'")
        payload["expr"] = args.expr
        payload["result"] = symbolic_integral(args.expr, args.var)
        return payload

    if args.mode == "simplify":
        if not args.expr:
            raise ValueError("--expr is required for mode 'simplify'")
        payload["expr"] = args.expr
        payload["result"] = symbolic_simplify(args.expr)
        return payload

    if args.mode == "solve":
        if not args.equation:
            raise ValueError("--equation is required for mode 'solve'")
        payload["equation"] = args.equation
        payload["solutions"] = solve_equation(args.equation, args.var)
        return payload

    if args.mode == "eval":
        if not args.expr:
            raise ValueError("--expr is required for mode 'eval'")
        payload["expr"] = args.expr
        assignments = parse_assignments(args.value)
        payload["values"] = assignments
        payload["result"] = evaluate_expression(args.expr, assignments)
        return payload

    raise ValueError(f"Unhandled math mode: {args.mode}")


def handle_math(args: argparse.Namespace) -> int:
    try:
        result = _run_math_operation(args)
    except ValueError as exc:
        print(str(exc))
        return 2
    except Exception as exc:
        print(str(exc))
        return 1

    if args.json:
        print(json.dumps(result, indent=2))
        return 0

    print(f"Mode: {result['mode']}")
    if "expr" in result:
        print(f"Expression: {result['expr']}")
    if "equation" in result:
        print(f"Equation: {result['equation']}")
    if "values" in result and result["values"]:
        print(f"Values: {result['values']}")
    if "result" in result:
        print(f"Result: {result['result']}")
    if "solutions" in result:
        print("Solutions:")
        for solution in result["solutions"]:
            print(f"  {solution}")
    return 0
