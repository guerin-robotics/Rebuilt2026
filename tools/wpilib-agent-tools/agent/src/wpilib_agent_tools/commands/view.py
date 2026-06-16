"""Open logs in AdvantageScope."""

from __future__ import annotations

import argparse
import os
import platform
import shutil
import subprocess

from wpilib_agent_tools.lib.log_reader import ensure_log_file


def register_subparser(subparsers: argparse._SubParsersAction) -> None:
    parser = subparsers.add_parser("view", help="Open a log file in AdvantageScope if available.")
    parser.add_argument("--file", help="Path to log file (defaults to latest).")
    parser.set_defaults(handler=handle_view)


def _find_advantagescope_app() -> str | None:
    candidates = [
        "/Applications/AdvantageScope.app",
        os.path.expanduser("~/Applications/AdvantageScope.app"),
    ]
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    return None


def handle_view(args: argparse.Namespace) -> int:
    try:
        file_path = ensure_log_file(args.file)
    except FileNotFoundError as exc:
        print(str(exc))
        return 1

    system = platform.system().lower()
    if system == "darwin":
        app_path = _find_advantagescope_app()
        if app_path:
            subprocess.run(["open", "-a", app_path, file_path], check=False)
            print(f"Opened {file_path} in AdvantageScope")
            return 0
        subprocess.run(["open", file_path], check=False)
        print(f"Opened {file_path}")
        return 0

    opener = "xdg-open" if shutil.which("xdg-open") else None
    if system == "windows":
        os.startfile(file_path)  # type: ignore[attr-defined]
        print(f"Opened {file_path}")
        return 0
    if opener:
        subprocess.run([opener, file_path], check=False)
        print(f"Opened {file_path}")
        return 0

    print("Could not find a suitable opener for this platform.")
    return 1
