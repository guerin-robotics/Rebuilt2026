"""Sandbox management commands."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from wpilib_agent_tools.commands import TOP_LEVEL_COMMANDS
from wpilib_agent_tools.lib.sandbox_manager import SandboxError, SandboxManager, format_sandbox_row
from wpilib_agent_tools.lib.output import emit


INTERNAL_COMMANDS = frozenset(TOP_LEVEL_COMMANDS)


def register_subparser(subparsers: argparse._SubParsersAction) -> None:
    parser = subparsers.add_parser("sandbox", help="Manage isolated sandboxes.")
    sandbox_subparsers = parser.add_subparsers(dest="sandbox_command", required=True)

    create_parser = sandbox_subparsers.add_parser("create", help="Create sandbox.")
    create_parser.add_argument("--name", required=True, help="Sandbox name.")
    create_parser.add_argument("--source", default="workspace", help="workspace | branch:<name> | rev:<sha>")
    create_parser.add_argument("--force", action="store_true", help="Replace sandbox if it already exists.")
    create_parser.add_argument("--json", action="store_true", help="Emit JSON output.")
    create_parser.set_defaults(handler=handle_create)

    list_parser = sandbox_subparsers.add_parser("list", help="List sandboxes.")
    list_parser.add_argument("--json", action="store_true", help="Emit JSON output.")
    list_parser.set_defaults(handler=handle_list)

    run_parser = sandbox_subparsers.add_parser("run", help="Run a command in sandbox.")
    run_parser.add_argument("--name", required=True, help="Sandbox name.")
    run_parser.add_argument("--detach", action="store_true", help="Run in background and return immediately.")
    run_parser.add_argument("--verbose", action="store_true", help="Stream child output directly.")
    run_parser.add_argument(
        "--max-lines",
        type=int,
        default=120,
        help="Maximum output lines returned when not verbose (<=0 means unlimited).",
    )
    run_parser.add_argument(
        "--tail",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="When truncating output, keep trailing lines (default: true).",
    )
    run_parser.add_argument("--include", help="Optional regex filter for returned output lines.")
    run_parser.add_argument("--json", action="store_true", help="Emit JSON output.")
    run_parser.add_argument("--json-compact", action="store_true", help="Emit compact JSON output.")
    run_parser.add_argument("command", nargs=argparse.REMAINDER, help="Command to run after `--`.")
    run_parser.set_defaults(handler=handle_run)

    status_parser = sandbox_subparsers.add_parser("status", help="Show sandbox status.")
    status_parser.add_argument("--name", help="Specific sandbox name; omit for all.")
    status_parser.add_argument("--json", action="store_true", help="Emit JSON output.")
    status_parser.set_defaults(handler=handle_status)

    stop_parser = sandbox_subparsers.add_parser("stop", help="Stop active sandbox job.")
    stop_parser.add_argument("--name", required=True, help="Sandbox name.")
    stop_parser.add_argument("--force", action="store_true", help="Escalate to SIGKILL if still running.")
    stop_parser.add_argument("--json", action="store_true", help="Emit JSON output.")
    stop_parser.set_defaults(handler=handle_stop)

    clean_parser = sandbox_subparsers.add_parser("clean", help="Delete sandbox(es).")
    clean_parser.add_argument("--name", help="Delete one sandbox by name.")
    clean_parser.add_argument("--all", action="store_true", help="Delete all sandboxes.")
    clean_parser.add_argument("--older-than", type=float, help="Only clean sandboxes older than N hours.")
    clean_parser.add_argument("--force", action="store_true", help="Delete even if busy.")
    clean_parser.add_argument("--json", action="store_true", help="Emit JSON output.")
    clean_parser.set_defaults(handler=handle_clean)

    patch_parser = sandbox_subparsers.add_parser("patch", help="Generate patch from sandbox.")
    patch_parser.add_argument("--name", required=True, help="Sandbox name.")
    patch_parser.add_argument("--output", help="Write patch to a file.")
    patch_parser.add_argument("--json", action="store_true", help="Emit JSON output.")
    patch_parser.set_defaults(handler=handle_patch)


def _emit(payload: object, as_json: bool, *, compact_json: bool = False) -> None:
    emit(payload, as_json=as_json, compact_json=compact_json)


def _manager() -> SandboxManager:
    return SandboxManager(workspace_root=Path.cwd())


def handle_create(args: argparse.Namespace) -> int:
    manager = _manager()
    try:
        payload = manager.create(name=args.name, source_spec=args.source, force=args.force)
    except (SandboxError, RuntimeError) as exc:
        print(str(exc))
        return 1
    _emit(payload, args.json, compact_json=getattr(args, "json_compact", False))
    return 0


def handle_list(args: argparse.Namespace) -> int:
    manager = _manager()
    payload = manager.list()
    if args.json:
        _emit(payload, True)
        return 0
    if not payload:
        print("No sandboxes found.")
        return 0
    print(f"{'NAME':<20} {'STATUS':<20} PATH")
    print("-" * 96)
    for item in payload:
        print(format_sandbox_row(item))
    return 0


def _prepare_command(command: list[str]) -> list[str]:
    prepared = command[:]
    if prepared and prepared[0] == "--":
        prepared = prepared[1:]
    if not prepared:
        raise SandboxError("sandbox run requires a command after `--`")
    if prepared[0] in INTERNAL_COMMANDS:
        prepared = [sys.executable, "-m", "wpilib_agent_tools", *prepared]
    return prepared


def handle_run(args: argparse.Namespace) -> int:
    manager = _manager()
    try:
        command = _prepare_command(args.command)
        payload = manager.run(
            name=args.name,
            command=command,
            detach=args.detach,
            verbose=args.verbose,
            max_lines=args.max_lines,
            tail=args.tail,
            include=args.include,
        )
    except (SandboxError, RuntimeError) as exc:
        print(str(exc))
        return 1
    if args.json:
        _emit(payload, True, compact_json=args.json_compact)
    elif args.detach:
        print(f"Started detached job pid={payload.get('pid')} in sandbox '{args.name}'")
    elif args.verbose:
        print(
            f"Completed sandbox run in '{args.name}' "
            f"(exit_code={payload.get('exit_code')})."
        )
    else:
        summary = payload.get("output_summary") or {}
        print(
            f"Completed sandbox run in '{args.name}' "
            f"(exit_code={payload.get('exit_code')}, lines={summary.get('included_lines', 0)})."
        )
        excerpt = payload.get("output_excerpt") or []
        if excerpt:
            print("Output excerpt:")
            for line in excerpt:
                print(f"  {line}")
        artifact = payload.get("output_artifact")
        if artifact:
            print(f"Full output saved to: {artifact}")
    return int(payload.get("exit_code", 0))


def handle_status(args: argparse.Namespace) -> int:
    manager = _manager()
    try:
        if args.name:
            payload = manager.status(args.name)
        else:
            payload = [manager.status(item["name"]) for item in manager.list()]
    except SandboxError as exc:
        print(str(exc))
        return 1

    if args.json:
        _emit(payload, True, compact_json=getattr(args, "json_compact", False))
        return 0

    if isinstance(payload, list):
        if not payload:
            print("No sandboxes found.")
            return 0
        print(f"{'NAME':<20} {'STATUS':<20} PATH")
        print("-" * 96)
        for item in payload:
            print(format_sandbox_row(item))
        return 0

    print(json.dumps(payload, indent=2))
    return 0


def handle_stop(args: argparse.Namespace) -> int:
    manager = _manager()
    try:
        payload = manager.stop(name=args.name, force=args.force)
    except SandboxError as exc:
        print(str(exc))
        return 1
    _emit(payload, args.json, compact_json=getattr(args, "json_compact", False))
    return 0 if payload.get("stopped") else 1


def handle_clean(args: argparse.Namespace) -> int:
    manager = _manager()
    try:
        payload = manager.clean(
            all_sandboxes=args.all,
            name=args.name,
            older_than_hours=args.older_than,
            force=args.force,
        )
    except SandboxError as exc:
        print(str(exc))
        return 1
    _emit(payload, args.json, compact_json=getattr(args, "json_compact", False))
    return 0


def handle_patch(args: argparse.Namespace) -> int:
    manager = _manager()
    try:
        patch_text = manager.generate_patch(args.name)
    except SandboxError as exc:
        print(str(exc))
        return 1

    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(patch_text, encoding="utf-8")
        payload = {"name": args.name, "output": str(output_path), "bytes": len(patch_text.encode("utf-8"))}
        _emit(payload, args.json, compact_json=getattr(args, "json_compact", False))
    else:
        if args.json:
            _emit(
                {"name": args.name, "patch": patch_text},
                True,
                compact_json=getattr(args, "json_compact", False),
            )
        else:
            print(patch_text)
    return 0
