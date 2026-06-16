"""Cursor rule installation commands."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from wpilib_agent_tools.integrations import cursor_rules_dir

CORE_TEMPLATE_FILE = "core_always_on.mdc"
TOKEN_EFFICIENT_TEMPLATE_FILE = "token_efficient_always_on.mdc"

MODE_TEMPLATES: dict[str, list[str]] = {
    "core": [CORE_TEMPLATE_FILE],
    "all": [CORE_TEMPLATE_FILE, TOKEN_EFFICIENT_TEMPLATE_FILE],
}

OUTPUT_FILE_NAMES: dict[str, str] = {
    CORE_TEMPLATE_FILE: "wpilib-agent-tools-core.mdc",
    TOKEN_EFFICIENT_TEMPLATE_FILE: "wpilib-agent-tools-token-efficient.mdc",
}


def _template_dir() -> Path:
    return cursor_rules_dir()


def _read_template(template_name: str) -> str:
    path = _template_dir() / template_name
    if not path.exists():
        raise FileNotFoundError(f"Template not found: {path}")
    return path.read_text(encoding="utf-8")


def _resolve_rules_dir(args: argparse.Namespace) -> Path:
    if args.target == "workspace":
        return Path.cwd().resolve() / ".cursor" / "rules"

    if not args.output_dir:
        raise ValueError("--output-dir is required when --target custom is used")
    return Path(args.output_dir).expanduser().resolve()


def register_subparser(subparsers: argparse._SubParsersAction) -> None:
    parser = subparsers.add_parser("rules", help="Install Cursor rule templates.")
    rule_subparsers = parser.add_subparsers(dest="rules_command", required=True)

    install_parser = rule_subparsers.add_parser("install", help="Install rule templates.")
    install_parser.add_argument(
        "--mode",
        choices=["core", "all"],
        default="core",
        help="Rule install mode. 'core' installs the always-on core rule (default). 'all' also installs example rules (token-efficient agent usage).",
    )
    install_parser.add_argument(
        "--target",
        choices=["workspace", "custom"],
        default="workspace",
        help="Install target location.",
    )
    install_parser.add_argument(
        "--output-dir",
        help="Custom output directory for installed rule files (required with --target custom).",
    )
    install_parser.add_argument("--force", action="store_true", help="Overwrite existing rule files.")
    install_parser.add_argument("--json", action="store_true", help="Emit machine-readable JSON output.")
    install_parser.set_defaults(handler=handle_install)


def _emit(payload: dict[str, object], *, as_json: bool) -> None:
    if as_json:
        print(json.dumps(payload, indent=2))
        return

    print(f"Rules target: {payload['rules_dir']}")
    installed = payload["installed"]
    overwritten = payload["overwritten"]
    skipped = payload["skipped"]

    if installed:
        print("Installed:")
        for item in installed:  # type: ignore[assignment]
            print(f"  - {item}")
    if overwritten:
        print("Overwritten:")
        for item in overwritten:  # type: ignore[assignment]
            print(f"  - {item}")
    if skipped:
        print("Skipped:")
        for item in skipped:  # type: ignore[assignment]
            print(f"  - {item['path']} ({item['reason']})")


def handle_install(args: argparse.Namespace) -> int:
    try:
        rules_dir = _resolve_rules_dir(args)
    except ValueError as exc:
        print(str(exc))
        return 2

    rules_dir.mkdir(parents=True, exist_ok=True)
    selected_templates = MODE_TEMPLATES[args.mode]

    installed: list[str] = []
    overwritten: list[str] = []
    skipped: list[dict[str, str]] = []

    for template_name in selected_templates:
        content = _read_template(template_name)
        output_name = OUTPUT_FILE_NAMES[template_name]
        output_path = rules_dir / output_name
        output_str = str(output_path)

        if output_path.exists() and not args.force:
            skipped.append({"path": output_str, "reason": "exists (use --force to overwrite)"})
            continue
        if output_path.exists() and args.force:
            overwritten.append(output_str)
        else:
            installed.append(output_str)

        output_path.write_text(content, encoding="utf-8")

    payload: dict[str, object] = {
        "mode": args.mode,
        "target": args.target,
        "rules_dir": str(rules_dir),
        "installed": installed,
        "overwritten": overwritten,
        "skipped": skipped,
    }
    _emit(payload, as_json=args.json)
    return 0
