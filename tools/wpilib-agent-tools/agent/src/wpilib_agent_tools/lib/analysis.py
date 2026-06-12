"""Shared analysis helpers for log data."""

from __future__ import annotations

import ast
import math
import re
from dataclasses import dataclass
from typing import Any, Iterable

import numpy as np

Numeric = int | float
Point = tuple[float, Any]

EXPR_KEY_RE = re.compile(r"\{([^{}]+)\}")


@dataclass(frozen=True)
class MinMaxResult:
    """Min/Max values and their timestamps."""

    min_time: float
    min_value: float
    max_time: float
    max_value: float


@dataclass(frozen=True)
class StatsResult:
    """Summary statistics including common percentiles."""

    count: int
    mean: float
    stddev: float
    min: float
    p5: float
    p25: float
    p50: float
    p75: float
    p95: float
    max: float


@dataclass(frozen=True)
class ThresholdEvent:
    """A contiguous threshold crossing segment."""

    start_time: float
    end_time: float
    duration: float
    peak_value: float


@dataclass(frozen=True)
class FFTComponent:
    """Dominant FFT frequency component."""

    frequency_hz: float
    magnitude: float


@dataclass(frozen=True)
class SettleResult:
    """Setpoint step-response metrics."""

    step_start_time: float
    target_value: float
    rise_time: float | None
    settling_time: float | None
    overshoot_pct: float
    steady_state_error: float


def _as_numeric(value: Any) -> float | None:
    if isinstance(value, bool):
        return 1.0 if value else 0.0
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _numeric_points(points: Iterable[Point]) -> list[tuple[float, float]]:
    output: list[tuple[float, float]] = []
    for timestamp, value in points:
        numeric = _as_numeric(value)
        if numeric is None:
            continue
        output.append((float(timestamp), numeric))
    return output


def calculate_derivative(points: Iterable[Point]) -> list[tuple[float, float]]:
    """Compute derivative using finite differences."""
    point_list = list(points)
    if len(point_list) < 2:
        return []
    output: list[tuple[float, float]] = []
    for index in range(1, len(point_list)):
        prev_time, prev_value = point_list[index - 1]
        cur_time, cur_value = point_list[index]
        prev_num = _as_numeric(prev_value)
        cur_num = _as_numeric(cur_value)
        if prev_num is None or cur_num is None:
            continue
        dt = cur_time - prev_time
        if dt == 0:
            continue
        output.append((cur_time, (cur_num - prev_num) / dt))
    return output


def calculate_integral(points: Iterable[Point]) -> list[tuple[float, float]]:
    """Compute cumulative trapezoidal integral."""
    point_list = list(points)
    if len(point_list) < 2:
        return []
    output: list[tuple[float, float]] = []
    running = 0.0
    for index in range(1, len(point_list)):
        prev_time, prev_value = point_list[index - 1]
        cur_time, cur_value = point_list[index]
        prev_num = _as_numeric(prev_value)
        cur_num = _as_numeric(cur_value)
        if prev_num is None or cur_num is None:
            continue
        dt = cur_time - prev_time
        if dt == 0:
            continue
        running += ((prev_num + cur_num) / 2.0) * dt
        output.append((cur_time, running))
    return output


def calculate_average(points: Iterable[Point]) -> float | list[float] | None:
    """Compute scalar average or element-wise vector average."""
    values = [value for _, value in points]
    if not values:
        return None

    numeric_values = [_as_numeric(value) for value in values]
    if all(value is not None for value in numeric_values):
        numeric_list = [value for value in numeric_values if value is not None]
        if not numeric_list:
            return None
        return sum(numeric_list) / len(numeric_list)

    first = values[0]
    if isinstance(first, list):
        width = len(first)
        if width == 0:
            return None
        total = [0.0] * width
        count = 0
        for value in values:
            if not isinstance(value, list) or len(value) != width:
                continue
            row = [_as_numeric(item) for item in value]
            if any(item is None for item in row):
                continue
            for idx, item in enumerate(row):
                total[idx] += float(item)
            count += 1
        if count == 0:
            return None
        return [element / count for element in total]
    return None


def calculate_minmax(points: Iterable[Point]) -> MinMaxResult | None:
    """Compute min and max for scalar values."""
    numeric_points = _numeric_points(points)
    if not numeric_points:
        return None
    min_time, min_value = min(numeric_points, key=lambda pair: pair[1])
    max_time, max_value = max(numeric_points, key=lambda pair: pair[1])
    return MinMaxResult(
        min_time=min_time,
        min_value=min_value,
        max_time=max_time,
        max_value=max_value,
    )


def calculate_stats(points: Iterable[Point]) -> StatsResult | None:
    """Compute summary statistics for scalar values."""
    numeric_points = _numeric_points(points)
    if not numeric_points:
        return None
    values = np.array([value for _, value in numeric_points], dtype=float)
    return StatsResult(
        count=int(values.size),
        mean=float(np.mean(values)),
        stddev=float(np.std(values)),
        min=float(np.min(values)),
        p5=float(np.percentile(values, 5)),
        p25=float(np.percentile(values, 25)),
        p50=float(np.percentile(values, 50)),
        p75=float(np.percentile(values, 75)),
        p95=float(np.percentile(values, 95)),
        max=float(np.max(values)),
    )


def smooth_points(points: Iterable[Point], window: int) -> list[tuple[float, float]]:
    """Compute trailing moving-average smoothing."""
    if window <= 0:
        raise ValueError("--window must be greater than zero")
    numeric_points = _numeric_points(points)
    if not numeric_points:
        return []
    times = [timestamp for timestamp, _ in numeric_points]
    values = [value for _, value in numeric_points]
    smoothed: list[tuple[float, float]] = []
    running = 0.0
    for index, value in enumerate(values):
        running += value
        if index >= window:
            running -= values[index - window]
        span = min(index + 1, window)
        smoothed.append((times[index], running / span))
    return smoothed


def calculate_rms(points: Iterable[Point]) -> float | None:
    """Compute root-mean-square for scalar values."""
    numeric_points = _numeric_points(points)
    if not numeric_points:
        return None
    values = np.array([value for _, value in numeric_points], dtype=float)
    return float(np.sqrt(np.mean(values * values)))


def detect_threshold_events(
    points: Iterable[Point],
    *,
    above: float | None,
    below: float | None,
    min_duration: float = 0.0,
) -> list[ThresholdEvent]:
    """Find contiguous threshold events above/below target values."""
    if (above is None and below is None) or (above is not None and below is not None):
        raise ValueError("Specify exactly one of --above or --below")
    if min_duration < 0:
        raise ValueError("--min-duration must be non-negative")

    numeric_points = _numeric_points(points)
    if not numeric_points:
        return []

    def is_match(value: float) -> bool:
        if above is not None:
            return value >= above
        return value <= float(below)

    events: list[ThresholdEvent] = []
    active_start: float | None = None
    active_end: float | None = None
    active_peak: float | None = None

    for timestamp, value in numeric_points:
        if is_match(value):
            if active_start is None:
                active_start = timestamp
                active_peak = value
            active_end = timestamp
            if above is not None and active_peak is not None:
                active_peak = max(active_peak, value)
            elif active_peak is not None:
                active_peak = min(active_peak, value)
            continue

        if active_start is not None and active_end is not None and active_peak is not None:
            duration = active_end - active_start
            if duration >= min_duration:
                events.append(
                    ThresholdEvent(
                        start_time=active_start,
                        end_time=active_end,
                        duration=duration,
                        peak_value=active_peak,
                    )
                )
        active_start = None
        active_end = None
        active_peak = None

    if active_start is not None and active_end is not None and active_peak is not None:
        duration = active_end - active_start
        if duration >= min_duration:
            events.append(
                ThresholdEvent(
                    start_time=active_start,
                    end_time=active_end,
                    duration=duration,
                    peak_value=active_peak,
                )
            )

    return events


def calculate_fft(points: Iterable[Point], *, top_n: int = 5) -> list[FFTComponent]:
    """Return strongest non-DC FFT components."""
    if top_n <= 0:
        raise ValueError("--top must be greater than zero")
    numeric_points = _numeric_points(points)
    if len(numeric_points) < 4:
        return []

    times = np.array([timestamp for timestamp, _ in numeric_points], dtype=float)
    values = np.array([value for _, value in numeric_points], dtype=float)
    dts = np.diff(times)
    positive_dts = dts[dts > 0]
    if positive_dts.size == 0:
        return []
    dt = float(np.median(positive_dts))
    if dt <= 0:
        return []

    centered = values - np.mean(values)
    fft_values = np.fft.rfft(centered)
    freqs = np.fft.rfftfreq(centered.size, d=dt)
    mags = np.abs(fft_values)
    if mags.size == 0:
        return []
    mags[0] = 0.0  # Ignore DC component.

    order = np.argsort(mags)[::-1]
    components: list[FFTComponent] = []
    for index in order:
        magnitude = float(mags[index])
        if magnitude <= 0.0:
            continue
        components.append(FFTComponent(frequency_hz=float(freqs[index]), magnitude=magnitude))
        if len(components) >= top_n:
            break
    return components


def extract_expression_keys(expression: str) -> list[str]:
    """Extract `{Log/Key}` placeholders from an expression string."""
    seen: set[str] = set()
    keys: list[str] = []
    for match in EXPR_KEY_RE.finditer(expression):
        key = match.group(1).strip()
        if not key or key in seen:
            continue
        seen.add(key)
        keys.append(key)
    return keys


def _safe_expression(
    expression: str,
    key_to_var: dict[str, str],
    allowed_functions: set[str],
) -> ast.Expression:
    rewritten = expression
    for key, variable in key_to_var.items():
        rewritten = rewritten.replace(f"{{{key}}}", variable)

    tree = ast.parse(rewritten, mode="eval")
    allowed_nodes = (
        ast.Expression,
        ast.BinOp,
        ast.UnaryOp,
        ast.Call,
        ast.Name,
        ast.Load,
        ast.Constant,
        ast.Add,
        ast.Sub,
        ast.Mult,
        ast.Div,
        ast.Pow,
        ast.Mod,
        ast.FloorDiv,
        ast.USub,
        ast.UAdd,
    )
    for node in ast.walk(tree):
        if not isinstance(node, allowed_nodes):
            raise ValueError("Unsupported token in --expr; use arithmetic expressions only")
        if isinstance(node, ast.Call):
            if not isinstance(node.func, ast.Name) or node.func.id not in allowed_functions:
                raise ValueError("Unsupported function in --expr")
        if isinstance(node, ast.Name):
            if node.id not in allowed_functions and node.id not in key_to_var.values():
                raise ValueError(f"Unknown symbol in --expr: {node.id}")
    return tree


def evaluate_expression_series(
    expression: str,
    series_by_key: dict[str, list[Point]],
) -> list[tuple[float, float]]:
    """Evaluate `{key}` expression over aligned timestamps from multiple series."""
    keys = extract_expression_keys(expression)
    if not keys:
        raise ValueError("--expr must include at least one {Log/Key} placeholder")

    missing = [key for key in keys if key not in series_by_key]
    if missing:
        raise ValueError(f"Missing series for keys: {', '.join(missing)}")

    numeric_series: dict[str, dict[float, float]] = {}
    for key in keys:
        numeric_series[key] = {timestamp: value for timestamp, value in _numeric_points(series_by_key[key])}

    common_times: set[float] | None = None
    for key in keys:
        key_times = set(numeric_series[key].keys())
        common_times = key_times if common_times is None else common_times & key_times
    if not common_times:
        return []

    key_to_var = {key: f"k{idx}" for idx, key in enumerate(keys)}
    functions = {
        "abs": abs,
        "min": min,
        "max": max,
        "sqrt": math.sqrt,
        "sin": math.sin,
        "cos": math.cos,
        "tan": math.tan,
        "log": math.log,
        "exp": math.exp,
    }
    parsed = _safe_expression(expression, key_to_var, set(functions.keys()))
    code = compile(parsed, "<expr>", "eval")

    output: list[tuple[float, float]] = []
    for timestamp in sorted(common_times):
        context = {key_to_var[key]: numeric_series[key][timestamp] for key in keys}
        context.update(functions)
        try:
            value = eval(code, {"__builtins__": {}}, context)
        except Exception as exc:
            raise ValueError(f"Unable to evaluate --expr: {exc}") from exc
        numeric = _as_numeric(value)
        if numeric is None or not math.isfinite(numeric):
            continue
        output.append((timestamp, numeric))
    return output


def calculate_settle_metrics(
    points: Iterable[Point],
    *,
    tolerance: float,
    setpoint: float | None = None,
    setpoint_points: Iterable[Point] | None = None,
) -> SettleResult | None:
    """Calculate rise time, settling time, overshoot, and steady-state error."""
    if tolerance <= 0:
        raise ValueError("--tolerance must be greater than zero")
    actual_points = _numeric_points(points)
    if not actual_points:
        return None

    step_start: float
    target: float
    if setpoint_points is not None:
        target_series = _numeric_points(setpoint_points)
    else:
        target_series = []

    if target_series:
        initial = target_series[0][1]
        change_index = 0
        for idx in range(1, len(target_series)):
            if abs(target_series[idx][1] - initial) > 1e-9:
                change_index = idx
                break
        step_start = target_series[change_index][0]
        target = target_series[-1][1]
    elif setpoint is not None:
        step_start = actual_points[0][0]
        target = float(setpoint)
    else:
        raise ValueError("settle mode requires --setpoint or --setpoint-key")

    observed = [(timestamp, value) for timestamp, value in actual_points if timestamp >= step_start]
    if not observed:
        return None

    start_value = observed[0][1]
    rising = target >= start_value

    rise_time: float | None = None
    for timestamp, value in observed:
        crossed = value >= target if rising else value <= target
        if crossed:
            rise_time = timestamp - step_start
            break

    tolerance_abs = abs(target) * tolerance if abs(target) > 1e-9 else tolerance
    settling_time: float | None = None
    for idx, (timestamp, _) in enumerate(observed):
        if all(abs(value - target) <= tolerance_abs for _, value in observed[idx:]):
            settling_time = timestamp - step_start
            break

    if rising:
        peak = max(value for _, value in observed)
        overshoot = max(0.0, peak - target)
    else:
        trough = min(value for _, value in observed)
        overshoot = max(0.0, target - trough)
    denominator = max(abs(target), 1.0)
    overshoot_pct = (overshoot / denominator) * 100.0
    steady_state_error = observed[-1][1] - target

    return SettleResult(
        step_start_time=step_start,
        target_value=target,
        rise_time=rise_time,
        settling_time=settling_time,
        overshoot_pct=overshoot_pct,
        steady_state_error=steady_state_error,
    )


def reconstruct_driver_station_state(
    ds_series: dict[str, list[Point]],
    target_time: float | None = None,
) -> dict[str, Any]:
    """Reconstruct latest known DS state up to target time (or end)."""
    state: dict[str, Any] = {
        "DriverStation/Enabled": None,
        "DriverStation/Autonomous": None,
        "DriverStation/Test": None,
        "DriverStation/AllianceStation": None,
        "DriverStation/MatchTime": None,
    }
    for key in state:
        points = ds_series.get(key, [])
        for timestamp, value in points:
            if target_time is not None and timestamp > target_time:
                break
            state[key] = value
    return state
