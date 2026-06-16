"""Symbolic math helpers powered by sympy."""

from __future__ import annotations

from typing import Any

import sympy as sp


def symbolic_derivative(expression: str, variable: str) -> str:
    """Compute symbolic derivative d(expression)/d(variable)."""
    symbol = sp.Symbol(variable)
    return str(sp.simplify(sp.diff(sp.sympify(expression), symbol)))


def symbolic_integral(expression: str, variable: str) -> str:
    """Compute symbolic indefinite integral with respect to variable."""
    symbol = sp.Symbol(variable)
    return str(sp.simplify(sp.integrate(sp.sympify(expression), symbol)))


def symbolic_simplify(expression: str) -> str:
    """Simplify symbolic expression."""
    return str(sp.simplify(sp.sympify(expression)))


def solve_equation(equation: str, variable: str) -> list[str]:
    """Solve a single-variable equation for variable."""
    symbol = sp.Symbol(variable)
    if "=" in equation:
        lhs, rhs = equation.split("=", maxsplit=1)
        relation = sp.Eq(sp.sympify(lhs), sp.sympify(rhs))
    else:
        relation = sp.Eq(sp.sympify(equation), 0)
    solutions = sp.solve(relation, symbol)
    return [str(solution) for solution in solutions]


def evaluate_expression(expression: str, values: dict[str, str]) -> float | str:
    """Numerically evaluate expression with symbol substitutions."""
    parsed = sp.sympify(expression)
    substitutions = {sp.Symbol(name): sp.sympify(value) for name, value in values.items()}
    result = parsed.evalf(subs=substitutions)
    if result.free_symbols:
        missing = ", ".join(sorted(str(symbol) for symbol in result.free_symbols))
        raise ValueError(f"Expression still contains free symbols: {missing}")
    if result.is_real:
        return float(result)
    return str(result)


def parse_assignments(values: list[str] | None) -> dict[str, str]:
    """Parse repeated name=value CLI assignments."""
    if not values:
        return {}
    assignments: dict[str, str] = {}
    for item in values:
        if "=" not in item:
            raise ValueError(f"Invalid assignment '{item}'. Use name=value format.")
        name, value = item.split("=", maxsplit=1)
        symbol = name.strip()
        if not symbol:
            raise ValueError(f"Invalid assignment '{item}'. Missing variable name.")
        assignments[symbol] = value.strip()
    return assignments
