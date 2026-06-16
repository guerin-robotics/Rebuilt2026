"""Record NetworkTables data."""

from __future__ import annotations

import argparse
import json

from wpilib_agent_tools.lib.nt_recorder import NTRecorder


def register_subparser(subparsers: argparse._SubParsersAction) -> None:
    parser = subparsers.add_parser("record", help="Record live NT4 data to WPILOG.")
    parser.add_argument("--address", default="localhost", help="NT4 server address (host or host:port).")
    parser.add_argument("--duration", type=float, required=True, help="Duration in seconds.")
    parser.add_argument(
        "--keys",
        action="append",
        help="Key prefix filter (repeatable). If omitted, records all keys.",
    )
    parser.add_argument("--output", help="Output file path (default: agent/logs/nt_record_TIMESTAMP.wpilog).")
    parser.add_argument("--json", action="store_true", help="Emit JSON output.")
    parser.set_defaults(handler=handle_record)


def handle_record(args: argparse.Namespace) -> int:
    recorder = NTRecorder(
        address=args.address,
        duration_sec=args.duration,
        key_prefixes=args.keys or [],
    )
    try:
        result = recorder.record(output_file=args.output)
    except RuntimeError as exc:
        print(str(exc))
        return 1

    payload = {
        "output_file": result.output_file,
        "address": result.address,
        "duration_sec": result.duration_sec,
        "topic_count": result.topic_count,
        "sample_count": result.sample_count,
    }
    if args.json:
        print(json.dumps(payload, indent=2))
    else:
        print(
            f"Recorded {result.sample_count} samples across {result.topic_count} topics "
            f"to {result.output_file}"
        )
    return 0
