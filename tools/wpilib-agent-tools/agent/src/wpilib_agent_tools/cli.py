"""Command-line entry point for WPILib Agent Tools."""

from __future__ import annotations

import argparse
from typing import Any, Callable

from wpilib_agent_tools import __version__
from wpilib_agent_tools.commands import TOP_LEVEL_COMMANDS
from wpilib_agent_tools.commands import (
    graph,
    harness,
    keys,
    logs,
    math,
    query,
    record,
    rules,
    sandbox,
    sim,
    view,
)


CommandRegistrar = Callable[[Any], None]


def build_parser() -> argparse.ArgumentParser:
    """Build and return the top-level CLI parser."""
    parser = argparse.ArgumentParser(
        prog="wpilib-agent-tools",
        description="Sandbox-first tooling for WPILib robot iteration and analysis.",
    )
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    subparsers = parser.add_subparsers(dest="command", required=True)

    registrars: dict[str, CommandRegistrar] = {
        "harness": harness.register_subparser,
        "sim": sim.register_subparser,
        "logs": logs.register_subparser,
        "keys": keys.register_subparser,
        "query": query.register_subparser,
        "math": math.register_subparser,
        "graph": graph.register_subparser,
        "record": record.register_subparser,
        "view": view.register_subparser,
        "sandbox": sandbox.register_subparser,
        "rules": rules.register_subparser,
    }
    unknown = sorted(set(registrars) - set(TOP_LEVEL_COMMANDS))
    missing = sorted(set(TOP_LEVEL_COMMANDS) - set(registrars))
    if unknown or missing:
        raise RuntimeError(
            "Command registry mismatch: "
            f"unknown={unknown or 'none'} missing={missing or 'none'}"
        )
    for command_name in TOP_LEVEL_COMMANDS:
        registrars[command_name](subparsers)

    return parser


def main(argv: list[str] | None = None) -> int:
    """Program entry point."""
    parser = build_parser()
    args = parser.parse_args(argv)
    handler = getattr(args, "handler", None)
    if handler is None:
        parser.print_help()
        return 2
    return int(handler(args) or 0)
