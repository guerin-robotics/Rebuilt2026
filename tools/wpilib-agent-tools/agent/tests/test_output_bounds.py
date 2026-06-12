from __future__ import annotations

from wpilib_agent_tools.commands.query import _summarize_result_item, _trim_result_item
from wpilib_agent_tools.lib.output import bound_lines


def test_bound_lines_tail_truncates() -> None:
    lines = ["a", "b", "c", "d"]
    bounded, meta = bound_lines(lines, max_lines=2, tail=True)
    assert bounded == ["c", "d"]
    assert meta["truncated"] is True
    assert meta["returned_lines"] == 2


def test_trim_result_item_limits_lists() -> None:
    item = {
        "file": "x",
        "mode": "values",
        "values": [(0.0, 1), (1.0, 2), (2.0, 3)],
        "series": {"DriverStation/Enabled": [(0.0, True), (1.0, False), (2.0, True)]},
    }
    trimmed = _trim_result_item(item, max_items=2, tail=True)
    assert trimmed["values"] == [(1.0, 2), (2.0, 3)]
    assert trimmed["series"]["DriverStation/Enabled"] == [(1.0, False), (2.0, True)]
    assert trimmed["output_summary"]["values"]["truncated"] is True


def test_summarize_result_item_counts_series() -> None:
    item = {
        "file": "x",
        "mode": "ds",
        "state": {"DriverStation/Enabled": True},
        "series": {"DriverStation/Enabled": [(0.0, False), (1.0, True)]},
    }
    summary = _summarize_result_item(item)
    assert "series" not in summary
    assert summary["series_counts"]["DriverStation/Enabled"] == 2
