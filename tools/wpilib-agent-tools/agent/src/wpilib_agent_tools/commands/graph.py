"""Graph log values."""

from __future__ import annotations

import argparse
import importlib
import json
from pathlib import Path
from typing import Any

from wpilib_agent_tools.lib.analysis import calculate_derivative, calculate_integral
from wpilib_agent_tools.lib.log_reader import LogReader, ensure_log_file


def register_subparser(subparsers: argparse._SubParsersAction) -> None:
    parser = subparsers.add_parser("graph", help="Generate graphs from log data.")
    parser.add_argument("--file", help="Path to log file (defaults to latest).")
    parser.add_argument(
        "--key",
        action="append",
        required=True,
        help="Key to graph. Repeat for multiple series.",
    )
    parser.add_argument(
        "--mode",
        choices=["values", "deriv", "integral"],
        default="values",
        help="Graph transformation mode.",
    )
    parser.add_argument("--output", default="graph.png", help="Output PNG filename.")
    parser.add_argument("--start", type=float, help="Start timestamp seconds.")
    parser.add_argument("--end", type=float, help="End timestamp seconds.")
    parser.add_argument("--title", help="Custom graph title.")
    parser.add_argument("--scatter", action="store_true", help="Use scatter plot.")
    parser.add_argument("--json", action="store_true", help="Emit JSON output.")
    parser.set_defaults(handler=handle_graph)


def _numeric_points(points: list[tuple[float, Any]]) -> tuple[list[tuple[float, float]], int]:
    output: list[tuple[float, float]] = []
    skipped_non_numeric = 0
    for timestamp, value in points:
        if isinstance(value, bool):
            output.append((timestamp, 1.0 if value else 0.0))
        elif isinstance(value, (int, float)):
            output.append((timestamp, float(value)))
        elif isinstance(value, list) and value and isinstance(value[0], (int, float)):
            output.append((timestamp, float(value[0])))
        else:
            skipped_non_numeric += 1
    return output, skipped_non_numeric


def _expand_keys(keys: list[str]) -> list[str]:
    expanded: list[str] = []
    for key in keys:
        # Accept both repeated --key flags and comma-separated key lists.
        for piece in key.split(","):
            normalized = piece.strip()
            if normalized:
                expanded.append(normalized)
    return expanded


def _load_pyplot() -> Any:
    """Load matplotlib lazily so non-graph commands avoid GUI/font side effects."""
    try:
        matplotlib = importlib.import_module("matplotlib")
        matplotlib.use("Agg")
        return importlib.import_module("matplotlib.pyplot")
    except Exception as exc:
        raise RuntimeError(
            "matplotlib is required for graph command. Install matplotlib to enable plotting."
        ) from exc


def handle_graph(args: argparse.Namespace) -> int:
    try:
        file_path = ensure_log_file(args.file)
    except FileNotFoundError as exc:
        print(str(exc))
        return 1

    reader = LogReader(file_path)
    plot_rows: list[dict[str, Any]] = []
    skipped_non_numeric_by_key: dict[str, int] = {}

    requested_keys = _expand_keys(args.key)
    for key in requested_keys:
        points, skipped_non_numeric = _numeric_points(
            reader.read_key_points(key, start=args.start, end=args.end)
        )
        if skipped_non_numeric > 0:
            skipped_non_numeric_by_key[key] = skipped_non_numeric
        if args.mode == "deriv":
            points = calculate_derivative(points)
        elif args.mode == "integral":
            points = calculate_integral(points)
        if not points:
            continue
        plot_rows.append({"key": key, "points": points})
    plotted_keys = {row["key"] for row in plot_rows}
    skipped_empty_keys = [key for key in requested_keys if key not in plotted_keys]

    if not plot_rows:
        if args.json:
            print(
                json.dumps(
                    {
                        "file": file_path,
                        "mode": args.mode,
                        "series_count": 0,
                        "requested_keys": requested_keys,
                        "skipped_empty_keys": skipped_empty_keys,
                        "skipped_non_numeric_by_key": skipped_non_numeric_by_key,
                    },
                    indent=2,
                )
            )
        else:
            print("No graphable data found for requested keys.")
            if skipped_empty_keys:
                print("Keys with no graphable samples:")
                for key in skipped_empty_keys:
                    print(f"  {key}")
            if skipped_non_numeric_by_key:
                print("Skipped non-numeric samples:")
                for key, count in skipped_non_numeric_by_key.items():
                    print(f"  {key}: {count}")
        return 1

    try:
        plt = _load_pyplot()
    except RuntimeError as exc:
        print(str(exc))
        return 1

    plt.figure(figsize=(12, 6))
    for row in plot_rows:
        xs = [point[0] for point in row["points"]]
        ys = [point[1] for point in row["points"]]
        label = row["key"] if args.mode == "values" else f"{args.mode}({row['key']})"
        if args.scatter:
            plt.scatter(xs, ys, s=8, label=label)
        else:
            plt.plot(xs, ys, label=label)

    title = args.title or f"WPILib Agent Tools Graph ({args.mode})"
    plt.title(title)
    plt.xlabel("Time (s)")
    plt.ylabel("Value")
    plt.legend()
    plt.grid(True, linestyle="--", alpha=0.5)

    output_dir = Path("agent/visualizations")
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / Path(args.output).name
    plt.tight_layout()
    plt.savefig(output_path)
    plt.close()

    payload = {
        "file": file_path,
        "mode": args.mode,
        "output": str(output_path),
        "series_count": len(plot_rows),
        "requested_keys": requested_keys,
        "skipped_empty_keys": skipped_empty_keys,
        "skipped_non_numeric_by_key": skipped_non_numeric_by_key,
    }
    if args.json:
        print(json.dumps(payload, indent=2))
    else:
        print(f"Graph saved to: {output_path}")
        if skipped_empty_keys:
            print("Keys with no graphable samples:")
            for key in skipped_empty_keys:
                print(f"  {key}")
        if skipped_non_numeric_by_key:
            print("Skipped non-numeric samples:")
            for key, count in skipped_non_numeric_by_key.items():
                print(f"  {key}: {count}")
    return 0
