"""Sandbox lifecycle and orchestration."""

from __future__ import annotations

import json
import os
import re
import shutil
import signal
import subprocess
import time
from collections import deque
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterator

from wpilib_agent_tools.lib.source_resolver import (
    ResolvedSource,
    SourceResolutionError,
    copy_untracked_files,
    resolve_source,
)


class SandboxError(RuntimeError):
    """Raised for sandbox lifecycle failures."""


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _run(cmd: list[str], cwd: Path | None = None, *, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        cmd,
        cwd=str(cwd) if cwd else None,
        text=True,
        capture_output=True,
        check=check,
    )


def _is_process_running(pid: int | None) -> bool:
    if pid is None:
        return False
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


@dataclass
class SandboxPaths:
    """Filesystem locations used by sandbox manager."""

    root: Path
    sandboxes: Path
    metadata: Path
    locks: Path


class SandboxManager:
    """Create/run/inspect/stop/clean isolated sandboxes."""

    NAME_PATTERN = re.compile(r"^[a-zA-Z0-9][a-zA-Z0-9._-]{0,63}$")
    STALE_LOCK_MAX_AGE_SEC = 6 * 3600

    def __init__(self, workspace_root: str | Path | None = None) -> None:
        self.workspace_root = Path(workspace_root or os.getcwd()).resolve()
        root = Path.home() / ".wpilib-agent-tools"
        self.paths = SandboxPaths(
            root=root,
            sandboxes=root / "sandboxes",
            metadata=root / "metadata",
            locks=root / "locks",
        )
        self._ensure_dirs()

    def _ensure_dirs(self) -> None:
        self.paths.sandboxes.mkdir(parents=True, exist_ok=True)
        self.paths.metadata.mkdir(parents=True, exist_ok=True)
        self.paths.locks.mkdir(parents=True, exist_ok=True)

    def _validate_name(self, name: str) -> None:
        if not self.NAME_PATTERN.fullmatch(name):
            raise SandboxError(
                "Invalid sandbox name. Use letters, numbers, dot, underscore, hyphen (1-64 chars)."
            )

    def _sandbox_path(self, name: str) -> Path:
        return self.paths.sandboxes / name

    def _metadata_path(self, name: str) -> Path:
        return self.paths.metadata / f"{name}.json"

    def _lock_path(self, name: str) -> Path:
        return self.paths.locks / f"{name}.lock"

    def _parse_lock_file(self, lock_file: Path) -> tuple[int | None, float | None]:
        try:
            raw = lock_file.read_text(encoding="utf-8").strip()
        except OSError:
            return None, None
        if not raw:
            return None, None
        parts = raw.split()
        if len(parts) < 2:
            return None, None
        try:
            pid = int(parts[0])
        except ValueError:
            pid = None
        try:
            started = float(parts[1])
        except ValueError:
            started = None
        return pid, started

    def _reap_stale_lock(self, lock_file: Path) -> bool:
        if not lock_file.exists():
            return False
        pid, started = self._parse_lock_file(lock_file)
        now = time.time()
        age = (now - started) if started is not None else None

        if pid is not None and _is_process_running(pid):
            # Keep live locks unless they are extremely old.
            if age is None or age < self.STALE_LOCK_MAX_AGE_SEC:
                return False
        elif age is not None and age < self.STALE_LOCK_MAX_AGE_SEC:
            # If lock owner is unknown and lock is recent, avoid false unlock.
            return False

        try:
            lock_file.unlink()
            return True
        except FileNotFoundError:
            return False
        except OSError:
            return False

    def _read_metadata(self, name: str) -> dict[str, Any]:
        meta_file = self._metadata_path(name)
        if not meta_file.exists():
            raise SandboxError(f"Sandbox '{name}' does not exist")
        with meta_file.open("r", encoding="utf-8") as handle:
            return json.load(handle)

    def _write_metadata(self, name: str, payload: dict[str, Any]) -> None:
        payload["updated_at"] = _utc_now()
        meta_file = self._metadata_path(name)
        with meta_file.open("w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2)

    @contextmanager
    def _lock(self, name: str) -> Iterator[None]:
        lock_file = self._lock_path(name)
        fd: int | None = None
        for _attempt in range(2):
            try:
                fd = os.open(str(lock_file), os.O_CREAT | os.O_EXCL | os.O_WRONLY)
                break
            except FileExistsError as exc:
                if self._reap_stale_lock(lock_file):
                    continue
                raise SandboxError(
                    f"Sandbox '{name}' is currently locked by another operation."
                ) from exc
        if fd is None:
            raise SandboxError(
                f"Sandbox '{name}' is currently locked by another operation."
            )

        try:
            os.write(fd, f"{os.getpid()} {time.time()}\n".encode("utf-8"))
            os.close(fd)
            yield
        finally:
            try:
                lock_file.unlink()
            except FileNotFoundError:
                pass

    def _default_metadata(
        self,
        name: str,
        source: ResolvedSource,
        sandbox_path: Path,
        method: str,
    ) -> dict[str, Any]:
        return {
            "name": name,
            "path": str(sandbox_path),
            "created_at": _utc_now(),
            "updated_at": _utc_now(),
            "status": "idle",
            "source": {
                "requested": source.requested,
                "kind": source.kind,
                "repo_root": str(source.repo_root) if source.repo_root else None,
                "workspace_root": str(source.workspace_root),
                "base_revision": source.base_revision,
                "branch_name": source.branch_name,
                "git_available": source.git_available,
            },
            "provisioning_method": method,
            "active_job": None,
            "last_exit_code": None,
            "artifacts": {
                "logs": "agent/logs",
                "plots": "agent/visualizations",
                "reports": "agent/reports",
            },
        }

    def create(self, name: str, source_spec: str = "workspace", *, force: bool = False) -> dict[str, Any]:
        """Create a sandbox from workspace/branch/rev source."""
        self._validate_name(name)
        sandbox_path = self._sandbox_path(name)
        meta_path = self._metadata_path(name)

        with self._lock(name):
            if sandbox_path.exists() or meta_path.exists():
                if not force:
                    raise SandboxError(f"Sandbox '{name}' already exists. Use --force to replace it.")
                self._delete_locked(name=name, force=True, allow_missing_meta=True)

            source = resolve_source(source_spec, cwd=self.workspace_root)
            method = self._provision_sandbox(sandbox_path=sandbox_path, source=source)
            self._prepare_artifact_dirs(sandbox_path)

            metadata = self._default_metadata(name=name, source=source, sandbox_path=sandbox_path, method=method)
            self._write_metadata(name, metadata)
            return metadata

    def _prepare_artifact_dirs(self, sandbox_path: Path) -> None:
        for rel in ("agent/logs", "agent/visualizations", "agent/reports"):
            (sandbox_path / rel).mkdir(parents=True, exist_ok=True)

    def _provision_sandbox(self, *, sandbox_path: Path, source: ResolvedSource) -> str:
        if source.git_available and source.repo_root and source.base_revision:
            try:
                _run(
                    [
                        "git",
                        "-C",
                        str(source.repo_root),
                        "worktree",
                        "add",
                        "--detach",
                        str(sandbox_path),
                        source.base_revision,
                    ]
                )
                method = "worktree"
            except subprocess.CalledProcessError:
                method = self._fallback_clone(sandbox_path=sandbox_path, source=source)
        elif source.git_available and source.repo_root and source.kind in {"branch", "rev"}:
            method = self._fallback_clone(sandbox_path=sandbox_path, source=source)
        else:
            method = self._fallback_copy_workspace(sandbox_path=sandbox_path, source=source)

        if source.kind == "workspace" and source.repo_root:
            self._apply_workspace_state(
                source=source,
                sandbox_path=sandbox_path,
            )

        return method

    def _fallback_clone(self, *, sandbox_path: Path, source: ResolvedSource) -> str:
        if not source.repo_root:
            raise SandboxError("Clone fallback requires repo root")
        _run(["git", "clone", "--quiet", str(source.repo_root), str(sandbox_path)])
        if source.base_revision:
            _run(["git", "-C", str(sandbox_path), "checkout", "--detach", source.base_revision])
        return "clone"

    def _fallback_copy_workspace(self, *, sandbox_path: Path, source: ResolvedSource) -> str:
        shutil.copytree(
            source.workspace_root,
            sandbox_path,
            ignore=shutil.ignore_patterns(".git", ".gradle", "build", "__pycache__", ".cursor"),
        )
        return "copy"

    def _apply_workspace_state(self, *, source: ResolvedSource, sandbox_path: Path) -> None:
        if source.patch_text:
            try:
                subprocess.run(
                    ["git", "-C", str(sandbox_path), "apply", "--whitespace=nowarn"],
                    input=source.patch_text,
                    text=True,
                    capture_output=True,
                    check=True,
                )
            except subprocess.CalledProcessError as exc:
                raise SandboxError(
                    "Failed to apply workspace patch snapshot in sandbox.\n"
                    f"stdout: {exc.stdout}\n"
                    f"stderr: {exc.stderr}"
                ) from exc
        if source.untracked_files and source.repo_root:
            copy_untracked_files(source.repo_root, sandbox_path, source.untracked_files)

    def list(self) -> list[dict[str, Any]]:
        """List sandbox metadata records."""
        items: list[dict[str, Any]] = []
        for meta_file in sorted(self.paths.metadata.glob("*.json")):
            with meta_file.open("r", encoding="utf-8") as handle:
                payload = json.load(handle)
            payload["running"] = _is_process_running(
                (payload.get("active_job") or {}).get("pid") if isinstance(payload.get("active_job"), dict) else None
            )
            items.append(payload)
        items.sort(key=lambda item: item.get("created_at", ""), reverse=True)
        return items

    def status(self, name: str) -> dict[str, Any]:
        """Return status and active-job health for a sandbox."""
        meta = self._read_metadata(name)
        active = meta.get("active_job")
        if isinstance(active, dict):
            pid = active.get("pid")
            running = _is_process_running(pid if isinstance(pid, int) else None)
            active["running"] = running
            if running and active.get("started_at_epoch"):
                active["duration_sec"] = round(time.time() - float(active["started_at_epoch"]), 3)
            elif not running:
                meta["status"] = "idle"
                meta["active_job"] = None
                self._write_metadata(name, meta)
        return meta

    def run(
        self,
        name: str,
        command: list[str],
        *,
        detach: bool = False,
        verbose: bool = False,
        max_lines: int | None = 120,
        tail: bool = True,
        include: str | None = None,
        env: dict[str, str] | None = None,
    ) -> dict[str, Any]:
        """Run a command inside a sandbox, tracking lifecycle metadata."""
        if not command:
            raise SandboxError("sandbox run requires a command")
        include_re: re.Pattern[str] | None = None
        if include:
            try:
                include_re = re.compile(include)
            except re.error as exc:
                raise SandboxError(f"Invalid --include regex: {exc}") from exc
        with self._lock(name):
            meta = self._read_metadata(name)
            sandbox_path = Path(meta["path"])
            active = meta.get("active_job")
            if isinstance(active, dict) and _is_process_running(active.get("pid")):
                raise SandboxError(
                    f"Sandbox '{name}' already has an active job (pid={active.get('pid')})."
                )

            run_env = os.environ.copy()
            run_env.update(env or {})
            run_env["WPILIB_AGENT_TOOLS_SANDBOX"] = "1"
            run_env["WPILIB_AGENT_TOOLS_SANDBOX_NAME"] = name

            popen_kwargs: dict[str, Any] = {
                "cwd": str(sandbox_path),
                "env": run_env,
                "preexec_fn": os.setsid,
            }
            if not detach and not verbose:
                popen_kwargs.update(
                    {
                        "stdout": subprocess.PIPE,
                        "stderr": subprocess.STDOUT,
                        "text": True,
                        "bufsize": 1,
                    }
                )
            try:
                process = subprocess.Popen(command, **popen_kwargs)
            except FileNotFoundError as exc:
                raise SandboxError(
                    f"Command not found in sandbox '{name}': {command[0]}"
                ) from exc
            except OSError as exc:
                raise SandboxError(
                    f"Failed to launch command in sandbox '{name}': {exc}"
                ) from exc
            pgid = os.getpgid(process.pid)
            meta["status"] = "running"
            meta["active_job"] = {
                "pid": process.pid,
                "pgid": pgid,
                "command": command,
                "started_at": _utc_now(),
                "started_at_epoch": time.time(),
            }
            self._write_metadata(name, meta)

        if detach:
            return {
                "name": name,
                "path": str(sandbox_path),
                "pid": process.pid,
                "pgid": pgid,
                "detached": True,
            }

        output_excerpt: list[str] = []
        output_summary: dict[str, Any] | None = None
        output_artifact: str | None = None
        if verbose:
            return_code = process.wait()
        else:
            reports_dir = sandbox_path / "agent" / "reports"
            reports_dir.mkdir(parents=True, exist_ok=True)
            artifact_path = reports_dir / f"sandbox-run-{int(time.time() * 1000)}.log"
            output_artifact = str(artifact_path)

            selected_head: list[str] = []
            selected_tail: deque[str] = deque(maxlen=max_lines if max_lines and max_lines > 0 else None)
            total_lines = 0
            included_lines = 0
            with artifact_path.open("w", encoding="utf-8") as artifact:
                stream = process.stdout
                if stream is not None:
                    for raw_line in stream:
                        artifact.write(raw_line)
                        total_lines += 1
                        line = raw_line.rstrip("\n")
                        if include_re and not include_re.search(line):
                            continue
                        included_lines += 1
                        if max_lines is None or max_lines <= 0:
                            selected_head.append(line)
                            continue
                        if tail:
                            selected_tail.append(line)
                        elif len(selected_head) < max_lines:
                            selected_head.append(line)
            return_code = process.wait()
            if max_lines is None or max_lines <= 0:
                output_excerpt = selected_head
            elif tail:
                output_excerpt = list(selected_tail)
            else:
                output_excerpt = selected_head
            output_summary = {
                "total_lines": total_lines,
                "included_lines": included_lines,
                "returned_lines": len(output_excerpt),
                "truncated": included_lines > len(output_excerpt),
                "tail": tail,
                "max_lines": max_lines,
                "include": include,
                "artifact": output_artifact,
            }

        with self._lock(name):
            meta = self._read_metadata(name)
            meta["status"] = "idle"
            meta["active_job"] = None
            meta["last_exit_code"] = return_code
            self._write_metadata(name, meta)
        return {
            "name": name,
            "path": str(sandbox_path),
            "exit_code": return_code,
            "detached": False,
            "verbose": verbose,
            "output_excerpt": output_excerpt,
            "output_summary": output_summary,
            "output_artifact": output_artifact,
        }

    def stop(self, name: str, *, force: bool = False) -> dict[str, Any]:
        """Stop active process group for sandbox if present."""
        with self._lock(name):
            meta = self._read_metadata(name)
            active = meta.get("active_job")
            if not isinstance(active, dict):
                return {"name": name, "stopped": False, "reason": "no_active_job"}
            pid = active.get("pid")
            pgid = active.get("pgid")
            if not isinstance(pid, int) or not _is_process_running(pid):
                meta["status"] = "idle"
                meta["active_job"] = None
                self._write_metadata(name, meta)
                return {"name": name, "stopped": False, "reason": "already_stopped"}

            if isinstance(pgid, int):
                os.killpg(pgid, signal.SIGTERM)
            else:
                os.kill(pid, signal.SIGTERM)

            deadline = time.time() + 5.0
            while time.time() < deadline:
                if not _is_process_running(pid):
                    break
                time.sleep(0.1)

            if _is_process_running(pid) and force:
                if isinstance(pgid, int):
                    os.killpg(pgid, signal.SIGKILL)
                else:
                    os.kill(pid, signal.SIGKILL)

            stopped = not _is_process_running(pid)
            if stopped:
                meta["status"] = "idle"
                meta["active_job"] = None
                self._write_metadata(name, meta)
            return {"name": name, "stopped": stopped, "pid": pid}

    def delete(self, name: str, *, force: bool = False) -> dict[str, Any]:
        """Delete one sandbox and metadata."""
        with self._lock(name):
            return self._delete_locked(name=name, force=force, allow_missing_meta=False)

    def _delete_locked(self, *, name: str, force: bool, allow_missing_meta: bool) -> dict[str, Any]:
        """Delete sandbox assuming caller already holds lock."""
        meta_path = self._metadata_path(name)
        sandbox_path = self._sandbox_path(name)

        meta: dict[str, Any] | None = None
        if meta_path.exists():
            with meta_path.open("r", encoding="utf-8") as handle:
                meta = json.load(handle)
        elif not allow_missing_meta:
            raise SandboxError(f"Sandbox '{name}' does not exist")

        if meta is not None:
            active = meta.get("active_job")
            if isinstance(active, dict) and _is_process_running(active.get("pid")):
                if not force:
                    raise SandboxError(f"Sandbox '{name}' is busy. Stop it first or use --force.")
                pid = active.get("pid")
                pgid = active.get("pgid")
                if isinstance(pgid, int):
                    os.killpg(pgid, signal.SIGTERM)
                elif isinstance(pid, int):
                    os.kill(pid, signal.SIGTERM)
                deadline = time.time() + 5.0
                while isinstance(pid, int) and time.time() < deadline and _is_process_running(pid):
                    time.sleep(0.1)
                if isinstance(pid, int) and _is_process_running(pid):
                    if isinstance(pgid, int):
                        os.killpg(pgid, signal.SIGKILL)
                    else:
                        os.kill(pid, signal.SIGKILL)

            method = str(meta.get("provisioning_method", "copy"))
            repo_root = (meta.get("source") or {}).get("repo_root")
        else:
            method = "copy"
            repo_root = None

        if sandbox_path.exists():
            if method == "worktree" and repo_root:
                try:
                    cmd = [
                        "git",
                        "-C",
                        str(repo_root),
                        "worktree",
                        "remove",
                    ]
                    if force:
                        cmd.append("--force")
                    cmd.append(str(sandbox_path))
                    _run(cmd)
                except Exception:
                    shutil.rmtree(sandbox_path, ignore_errors=True)
            else:
                shutil.rmtree(sandbox_path, ignore_errors=True)

        if meta_path.exists():
            meta_path.unlink()
        return {"name": name, "deleted": True}

    def clean(
        self,
        *,
        all_sandboxes: bool = False,
        name: str | None = None,
        older_than_hours: float | None = None,
        force: bool = False,
    ) -> dict[str, Any]:
        """Clean one sandbox by name or many by age."""
        if not all_sandboxes and not name:
            raise SandboxError("Provide --name or --all for clean")

        if name:
            result = self.delete(name=name, force=force)
            return {"deleted": [result], "skipped": []}

        now = datetime.now(timezone.utc).timestamp()
        deleted: list[dict[str, Any]] = []
        skipped: list[dict[str, Any]] = []
        for meta in self.list():
            candidate_name = str(meta.get("name"))
            created_at = str(meta.get("created_at", ""))
            if older_than_hours is not None:
                try:
                    created_ts = datetime.fromisoformat(created_at).timestamp()
                except ValueError:
                    created_ts = now
                age_hours = (now - created_ts) / 3600.0
                if age_hours < older_than_hours:
                    skipped.append({"name": candidate_name, "reason": "too_new"})
                    continue
            try:
                deleted.append(self.delete(candidate_name, force=force))
            except SandboxError as exc:
                skipped.append({"name": candidate_name, "reason": str(exc)})
        return {"deleted": deleted, "skipped": skipped}

    def generate_patch(self, name: str) -> str:
        """Generate deterministic patch for sandbox changes."""
        meta = self._read_metadata(name)
        sandbox_path = Path(meta["path"])
        if not (sandbox_path / ".git").exists():
            raise SandboxError("Patch generation requires a git-backed sandbox")
        result = _run(["git", "-C", str(sandbox_path), "diff", "--binary"])
        return result.stdout


def format_sandbox_row(meta: dict[str, Any]) -> str:
    """Format a concise status row for human output."""
    name = meta.get("name", "?")
    status = meta.get("status", "unknown")
    path = meta.get("path", "")
    active = meta.get("active_job")
    if isinstance(active, dict) and active.get("pid"):
        status = f"{status} (pid={active.get('pid')})"
    return f"{name:<20} {status:<20} {path}"
