"""List keys in a log file."""

from __future__ import annotations

import argparse

from wpilib_agent_tools.lib.log_reader import LogReader, ensure_log_file
from wpilib_agent_tools.lib.output import bound_lines, emit


def register_subparser(subparsers: argparse._SubParsersAction) -> None:
    parser = subparsers.add_parser("keys", help="List keys in a WPILOG file.")
    parser.add_argument("--file", help="Path to log file (defaults to latest).")
    parser.add_argument("--filter", help="Case-insensitive substring filter.")
    parser.add_argument("--summary", action="store_true", help="Emit only key count and source file.")
    parser.add_argument("--max-lines", type=int, help="Maximum number of keys returned.")
    parser.add_argument(
        "--tail",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="When truncating, keep trailing keys (default: true).",
    )
    parser.add_argument("--json", action="store_true", help="Emit JSON output.")
    parser.add_argument("--json-compact", action="store_true", help="Emit compact JSON output.")
    parser.set_defaults(handler=handle_keys)


def handle_keys(args: argparse.Namespace) -> int:
    try:
        path = ensure_log_file(args.file)
    except FileNotFoundError as exc:
        print(str(exc))
        return 1

    reader = LogReader(path)
    keys = reader.list_keys(args.filter)
    if args.summary:
        payload = {"file": path, "count": len(keys)}
        if args.json:
            emit(payload, as_json=True, compact_json=args.json_compact)
        else:
            print(f"Reading keys from: {path}")
            print(f"Matching keys: {len(keys)}")
        return 0

    bounded_keys, bounds = bound_lines(keys, max_lines=args.max_lines, tail=args.tail)
    if args.json:
        emit(
            {"file": path, "count": len(keys), "keys": bounded_keys, "output_summary": bounds},
            as_json=True,
            compact_json=args.json_compact,
        )
        return 0

    print(f"Reading keys from: {path}")
    print("-" * 40)
    for key in bounded_keys:
        print(key)
    if bounds["truncated"]:
        direction = "tail" if args.tail else "head"
        print(f"... truncated to {bounds['returned_lines']} keys ({direction})")
    return 0
