"""NetworkTables recording utilities."""

from __future__ import annotations

import socket
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

try:  # pragma: no cover - runtime dependency
    import ntcore
except ImportError:  # pragma: no cover - runtime dependency
    ntcore = None  # type: ignore[assignment]

try:  # pragma: no cover - runtime dependency
    from wpiutil import DataLogWriter
except ImportError:  # pragma: no cover - runtime dependency
    DataLogWriter = None  # type: ignore[assignment]


def _coerce_value(value: Any) -> Any:
    if value is None:
        return None
    if isinstance(value, (bool, int, float, str, bytes)):
        return value
    if isinstance(value, (list, tuple)):
        return [_coerce_value(item) for item in value]
    if isinstance(value, bytearray):
        return bytes(value)
    if hasattr(value, "tolist"):
        try:
            return _coerce_value(value.tolist())
        except Exception:
            return str(value)
    if hasattr(value, "item"):
        try:
            return _coerce_value(value.item())
        except Exception:
            return str(value)
    return str(value)


def _extract_entry_value(entry: Any) -> Any:
    """Get an entry value across RobotPy API variations."""
    for method_name in ("getValue", "value"):
        method = getattr(entry, method_name, None)
        if callable(method):
            try:
                maybe = method()
                if hasattr(maybe, "isValid") and callable(maybe.isValid) and not maybe.isValid():
                    return None
                if hasattr(maybe, "value") and callable(maybe.value):
                    return _coerce_value(maybe.value())
                if hasattr(maybe, "getValue") and callable(maybe.getValue):
                    return _coerce_value(maybe.getValue())
                return _coerce_value(maybe)
            except Exception:
                continue
    return None


def _extract_value_payload(value_obj: Any) -> Any:
    """Convert an ntcore Value-like object into appendable data."""
    if value_obj is None:
        return None

    # RobotPy ntcore Value objects expose isValid()/value().
    valid = getattr(value_obj, "isValid", None)
    if callable(valid):
        try:
            if not bool(valid()):
                return None
        except Exception:
            return None

    candidate = value_obj
    for method_name in ("value", "getValue"):
        method = getattr(candidate, method_name, None)
        if callable(method):
            try:
                candidate = method()
            except Exception:
                continue
            break

    return _coerce_value(candidate)


def _normalize_type(type_str: str | None, value: Any | None = None) -> str:
    text = (type_str or "").strip()
    normalized = text.lower()

    if normalized.startswith("struct:"):
        type_name = text.split(":", 1)[1].strip()
        if type_name:
            return f"struct:{type_name}"
    if normalized.startswith("proto:"):
        type_name = text.split(":", 1)[1].strip()
        if type_name:
            return f"proto:{type_name}"

    aliases = {
        "bool": "boolean",
        "bool[]": "boolean[]",
        "int": "int64",
        "int[]": "int64[]",
        "integer": "int64",
        "integer[]": "int64[]",
        "json": "string",
    }
    if normalized in aliases:
        return aliases[normalized]
    if normalized in {
        "double",
        "float",
        "int64",
        "boolean",
        "string",
        "double[]",
        "float[]",
        "int64[]",
        "boolean[]",
        "string[]",
        "raw",
    }:
        return normalized

    if isinstance(value, bytes):
        return "raw"
    if isinstance(value, bool):
        return "boolean"
    if isinstance(value, int):
        return "int64"
    if isinstance(value, float):
        return "double"
    if isinstance(value, str):
        return "string"
    if isinstance(value, (list, tuple)):
        items = list(value)
        if not items:
            return "raw"
        if all(isinstance(item, bool) for item in items):
            return "boolean[]"
        if all(isinstance(item, int) and not isinstance(item, bool) for item in items):
            return "int64[]"
        if all(isinstance(item, (int, float)) and not isinstance(item, bool) for item in items):
            return "double[]"
        if all(isinstance(item, str) for item in items):
            return "string[]"
    return "raw"


def _ensure_entry_id(
    *,
    log: Any,
    entry_ids: dict[str, int],
    topic_types: dict[str, str],
    name: str,
    type_str: str | None,
    value: Any | None = None,
) -> tuple[int, str]:
    normalized = _normalize_type(type_str, value)
    existing_type = topic_types.get(name)
    if existing_type is None:
        topic_types[name] = normalized
    normalized = topic_types.get(name, normalized)
    if name not in entry_ids:
        entry_ids[name] = int(log.start(name, normalized))
    return entry_ids[name], normalized


def _to_raw_bytes(value: Any) -> bytes:
    if isinstance(value, bytes):
        return value
    if isinstance(value, bytearray):
        return bytes(value)
    if isinstance(value, str):
        return value.encode("utf-8")
    if isinstance(value, (list, tuple)):
        try:
            return bytes(value)
        except Exception:
            return str(value).encode("utf-8")
    return str(value).encode("utf-8")


def _append_to_log(
    *,
    log: Any,
    entry_id: int,
    value: Any,
    type_str: str,
    timestamp_us: int,
) -> bool:
    normalized = _normalize_type(type_str, value)
    if value is None:
        return False
    try:
        if normalized == "double":
            log.appendDouble(entry_id, float(value), timestamp_us)
            return True
        if normalized == "float":
            log.appendFloat(entry_id, float(value), timestamp_us)
            return True
        if normalized == "int64":
            log.appendInteger(entry_id, int(value), timestamp_us)
            return True
        if normalized == "boolean":
            log.appendBoolean(entry_id, bool(value), timestamp_us)
            return True
        if normalized == "string":
            log.appendString(entry_id, str(value), timestamp_us)
            return True
        if normalized == "double[]":
            values = [float(item) for item in list(value)]
            log.appendDoubleArray(entry_id, values, timestamp_us)
            return True
        if normalized == "float[]":
            values = [float(item) for item in list(value)]
            log.appendFloatArray(entry_id, values, timestamp_us)
            return True
        if normalized == "int64[]":
            values = [int(item) for item in list(value)]
            log.appendIntegerArray(entry_id, values, timestamp_us)
            return True
        if normalized == "boolean[]":
            values = [bool(item) for item in list(value)]
            log.appendBooleanArray(entry_id, values, timestamp_us)
            return True
        if normalized == "string[]":
            values = [str(item) for item in list(value)]
            log.appendStringArray(entry_id, values, timestamp_us)
            return True
    except Exception:
        # Preserve samples even if a type conversion unexpectedly fails.
        log.appendRaw(entry_id, _to_raw_bytes(value), timestamp_us)
        return True

    log.appendRaw(entry_id, _to_raw_bytes(value), timestamp_us)
    return True


def _parse_address(address: str) -> tuple[str, int | None]:
    """Parse server address as host, optionally with :port."""
    text = address.strip()
    if not text:
        raise RuntimeError("NT4 server address cannot be empty.")

    if text.startswith("["):
        bracket_end = text.find("]")
        if bracket_end == -1:
            raise RuntimeError(f"Invalid NT4 server address '{address}': missing closing ']'.")
        host = text[1:bracket_end].strip()
        remainder = text[bracket_end + 1 :].strip()
        if not host:
            raise RuntimeError(f"Invalid NT4 server address '{address}': missing host.")
        if not remainder:
            return host, None
        if not remainder.startswith(":"):
            raise RuntimeError(
                f"Invalid NT4 server address '{address}': expected ':<port>' after ']'."
            )
        port_text = remainder[1:].strip()
        if not port_text.isdigit():
            raise RuntimeError(f"Invalid NT4 server address '{address}': invalid port '{port_text}'.")
        port = int(port_text)
        if port < 1 or port > 65535:
            raise RuntimeError(f"Invalid NT4 server address '{address}': port must be 1-65535.")
        return host, port

    if ":" in text and text.count(":") == 1:
        host, port_text = text.rsplit(":", 1)
        host = host.strip()
        port_text = port_text.strip()
        if not host:
            raise RuntimeError(f"Invalid NT4 server address '{address}': missing host.")
        if port_text:
            if not port_text.isdigit():
                raise RuntimeError(f"Invalid NT4 server address '{address}': invalid port '{port_text}'.")
            port = int(port_text)
            if port < 1 or port > 65535:
                raise RuntimeError(f"Invalid NT4 server address '{address}': port must be 1-65535.")
            return host, port
        raise RuntimeError(f"Invalid NT4 server address '{address}': missing port after ':'.")

    return text, None


def _resolve_host_ips(host: str) -> set[str]:
    """Resolve host to IP strings for connection-target matching."""
    resolved: set[str] = set()
    try:
        infos = socket.getaddrinfo(host, None, family=socket.AF_UNSPEC, type=socket.SOCK_STREAM)
    except Exception:
        return resolved
    for _family, _socktype, _proto, _canonname, sockaddr in infos:
        if not sockaddr:
            continue
        try:
            resolved.add(str(sockaddr[0]))
        except Exception:
            continue
    return resolved


def _connection_matches_target(
    connection: Any,
    *,
    server_host: str,
    server_port: int | None,
    resolved_host_ips: set[str],
) -> bool:
    remote_ip = str(getattr(connection, "remote_ip", "") or "").strip()
    remote_port_raw = getattr(connection, "remote_port", None)
    try:
        remote_port = int(remote_port_raw) if remote_port_raw is not None else 0
    except Exception:
        remote_port = 0

    if server_port is not None and remote_port != server_port:
        return False
    if not remote_ip:
        return False
    if remote_ip in resolved_host_ips:
        return True
    return remote_ip == server_host


@dataclass(frozen=True)
class RecordingResult:
    """Return value from a completed recording."""

    output_file: str
    address: str
    duration_sec: float
    topic_count: int
    sample_count: int


class NTRecorder:
    """Best-effort polling recorder for NetworkTables values."""

    def __init__(
        self,
        *,
        address: str,
        duration_sec: float,
        key_prefixes: list[str] | None = None,
        output_dir: str | Path = "agent/logs",
        poll_interval_sec: float = 0.02,
    ) -> None:
        self.address = address
        self.duration_sec = duration_sec
        self.key_prefixes = key_prefixes or []
        self.output_dir = Path(output_dir)
        self.poll_interval_sec = poll_interval_sec

    def _match_key(self, key: str) -> bool:
        # Preserve struct/proto schema definitions even when key filters are set.
        if "/.schema/" in key:
            return True
        if not self.key_prefixes:
            return True
        lowered = key.lower()
        for prefix in self.key_prefixes:
            if lowered.startswith(prefix.lower()):
                return True
        return False

    def _wait_for_connection(
        self,
        inst: Any,
        *,
        server_host: str,
        server_port: int | None,
    ) -> None:
        # Give NT4 enough time to establish transport before sampling. Very short
        # recordings still need a practical connection grace period on slower CI.
        timeout = max(2.0, min(5.0, self.duration_sec))
        deadline = time.monotonic() + timeout
        resolved_host_ips = _resolve_host_ips(server_host)

        while time.monotonic() < deadline:
            try:
                connections = list(inst.getConnections())
                if any(
                    _connection_matches_target(
                        connection,
                        server_host=server_host,
                        server_port=server_port,
                        resolved_host_ips=resolved_host_ips,
                    )
                    for connection in connections
                ):
                    return
                if not resolved_host_ips and server_port is None and inst.isConnected():
                    return
            except Exception:
                pass
            time.sleep(0.05)

        endpoint = server_host if server_port is None else f"{server_host}:{server_port}"
        raise RuntimeError(
            f"Unable to connect to NT4 server at '{endpoint}' within {timeout:.1f}s."
        )

    def _record_with_listener(
        self,
        *,
        inst: Any,
        log: Any,
        start_mono: float,
        topic_types: dict[str, str],
        entry_ids: dict[str, int],
        last_values: dict[str, Any],
    ) -> int:
        prefixes = self.key_prefixes or [""]
        subscriber = ntcore.MultiSubscriber(inst, prefixes)
        poller = ntcore.NetworkTableListenerPoller(inst)
        event_flags = ntcore.EventFlags.kPublish | ntcore.EventFlags.kValueAll
        poller.addListener(subscriber, int(event_flags))
        sample_count = 0
        try:
            while True:
                elapsed = time.monotonic() - start_mono
                if elapsed >= self.duration_sec:
                    break

                for event in poller.readQueue():
                    data = getattr(event, "data", None)
                    if data is None:
                        continue

                    if isinstance(data, ntcore.TopicInfo):
                        name = str(data.name)
                        if not self._match_key(name):
                            continue
                        type_str = str(getattr(data, "type_str", "unknown"))
                        topic_types.setdefault(name, _normalize_type(type_str))
                        continue

                    if isinstance(data, ntcore.ValueEventData):
                        try:
                            name = str(data.topic.getName())
                        except Exception:
                            continue
                        if not self._match_key(name):
                            continue

                        value = _extract_value_payload(getattr(data, "value", None))
                        if value is None:
                            continue
                        if name in last_values and last_values[name] == value:
                            continue

                        topic_type = topic_types.get(name)
                        if topic_type is None:
                            type_method = getattr(data.topic, "getTypeString", None)
                            if callable(type_method):
                                try:
                                    topic_type = str(type_method())
                                except Exception:
                                    topic_type = None
                        entry_id, normalized_type = _ensure_entry_id(
                            log=log,
                            entry_ids=entry_ids,
                            topic_types=topic_types,
                            name=name,
                            type_str=topic_type,
                            value=value,
                        )
                        timestamp_us = max(1, int(round((time.monotonic() - start_mono) * 1_000_000)))
                        if not _append_to_log(
                            log=log,
                            entry_id=entry_id,
                            value=value,
                            type_str=normalized_type,
                            timestamp_us=timestamp_us,
                        ):
                            continue
                        last_values[name] = value
                        sample_count += 1
                        continue

                log.flush()
                time.sleep(self.poll_interval_sec)
        finally:
            try:
                poller.close()
            finally:
                subscriber.close()
        return sample_count

    def _record_with_topic_scan(
        self,
        *,
        inst: Any,
        log: Any,
        start_mono: float,
        topic_types: dict[str, str],
        entry_ids: dict[str, int],
        last_values: dict[str, Any],
    ) -> int:
        # Fallback path for environments without listener APIs.
        sample_count = 0
        while True:
            elapsed = time.monotonic() - start_mono
            if elapsed >= self.duration_sec:
                break

            try:
                topics = inst.getTopics()
            except Exception:
                topics = []

            for topic in topics:
                try:
                    name = str(topic.getName())
                except Exception:
                    continue
                if not self._match_key(name):
                    continue

                type_method = getattr(topic, "getTypeString", None)
                if callable(type_method):
                    try:
                        topic_type_text = str(type_method())
                    except Exception:
                        topic_type_text = "unknown"
                else:
                    topic_type_text = str(type_method or "unknown")
                topic_types.setdefault(name, _normalize_type(topic_type_text))
                entry = inst.getEntry(name)
                value = _extract_entry_value(entry)
                if value is None:
                    continue
                if name in last_values and last_values[name] == value:
                    continue
                type_method = getattr(topic, "getTypeString", None)
                if callable(type_method):
                    try:
                        topic_type = str(type_method())
                    except Exception:
                        topic_type = topic_types.get(name)
                else:
                    topic_type = topic_types.get(name)
                entry_id, normalized_type = _ensure_entry_id(
                    log=log,
                    entry_ids=entry_ids,
                    topic_types=topic_types,
                    name=name,
                    type_str=topic_type,
                    value=value,
                )
                timestamp_us = max(1, int(round((time.monotonic() - start_mono) * 1_000_000)))
                if not _append_to_log(
                    log=log,
                    entry_id=entry_id,
                    value=value,
                    type_str=normalized_type,
                    timestamp_us=timestamp_us,
                ):
                    continue
                last_values[name] = value
                sample_count += 1

            log.flush()
            time.sleep(self.poll_interval_sec)
        return sample_count

    def record(self, output_file: str | Path | None = None) -> RecordingResult:
        """Record values for the configured duration and write WPILOG output."""
        if ntcore is None:
            raise RuntimeError(
                "pyntcore is required for `record`. Install dependencies first."
            )
        if DataLogWriter is None:
            raise RuntimeError(
                "robotpy-wpiutil is required for WPILOG recording. Install dependencies first."
            )

        self.output_dir.mkdir(parents=True, exist_ok=True)
        if output_file is None:
            stamp = datetime.now(tz=timezone.utc).strftime("%Y%m%d_%H%M%S")
            output_path = self.output_dir / f"nt_record_{stamp}.wpilog"
        else:
            output_path = Path(output_file)
            if not output_path.is_absolute():
                output_path = self.output_dir / output_path
        output_path.parent.mkdir(parents=True, exist_ok=True)

        inst = ntcore.NetworkTableInstance.getDefault()
        inst.stopServer()
        inst.stopClient()
        inst.startClient4("wpilib-agent-tools-recorder")
        server_host, server_port = _parse_address(self.address)
        if server_port is None:
            inst.setServer(server_host)
        else:
            inst.setServer(server_host, server_port)
        self._wait_for_connection(inst, server_host=server_host, server_port=server_port)

        start_mono = time.monotonic()
        topic_types: dict[str, str] = {}
        entry_ids: dict[str, int] = {}
        last_values: dict[str, Any] = {}
        sample_count = 0
        log: Any | None = None
        try:
            log = DataLogWriter(str(output_path))
            if all(hasattr(ntcore, attr) for attr in ("MultiSubscriber", "NetworkTableListenerPoller", "TopicInfo", "ValueEventData")):
                sample_count = self._record_with_listener(
                    inst=inst,
                    log=log,
                    start_mono=start_mono,
                    topic_types=topic_types,
                    entry_ids=entry_ids,
                    last_values=last_values,
                )
            else:
                sample_count = self._record_with_topic_scan(
                    inst=inst,
                    log=log,
                    start_mono=start_mono,
                    topic_types=topic_types,
                    entry_ids=entry_ids,
                    last_values=last_values,
                )
        finally:
            if log is not None:
                log.flush()
                log.stop()
            inst.stopClient()

        return RecordingResult(
            output_file=str(output_path),
            address=self.address,
            duration_sec=round(time.monotonic() - start_mono, 6),
            topic_count=len(entry_ids),
            sample_count=sample_count,
        )
