"""Resolve source selection for sandbox creation."""

from __future__ import annotations

import os
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path


class SourceResolutionError(RuntimeError):
    """Raised when a source spec cannot be resolved."""


@dataclass(frozen=True)
class ResolvedSource:
    """Resolved source descriptor for sandbox provisioning."""

    requested: str
    kind: str
    workspace_root: Path
    repo_root: Path | None
    git_available: bool
    base_revision: str | None
    branch_name: str | None
    patch_text: str
    untracked_files: list[str]


def _run_git(args: list[str], cwd: Path, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["git", *args],
        cwd=str(cwd),
        check=check,
        text=True,
        capture_output=True,
    )


def _is_git_repo(path: Path) -> bool:
    try:
        result = _run_git(["rev-parse", "--is-inside-work-tree"], cwd=path, check=False)
    except FileNotFoundError as exc:
        raise SourceResolutionError("Git executable is required but was not found") from exc
    return result.returncode == 0 and result.stdout.strip() == "true"


def _repo_root(path: Path) -> Path | None:
    if not _is_git_repo(path):
        return None
    result = _run_git(["rev-parse", "--show-toplevel"], cwd=path)
    return Path(result.stdout.strip())


def _head_revision(repo_root: Path) -> str | None:
    result = _run_git(["rev-parse", "HEAD"], cwd=repo_root, check=False)
    if result.returncode != 0:
        return None
    return result.stdout.strip()


def _workspace_patch(repo_root: Path, head_revision: str | None) -> str:
    if head_revision:
        result = _run_git(["diff", "--binary", "HEAD"], cwd=repo_root)
        return result.stdout
    # Initial commit case: no HEAD yet.
    unstaged = _run_git(["diff", "--binary"], cwd=repo_root).stdout
    staged = _run_git(["diff", "--binary", "--cached"], cwd=repo_root).stdout
    return f"{staged}\n{unstaged}".strip()


def _untracked_files(repo_root: Path) -> list[str]:
    result = _run_git(["ls-files", "--others", "--exclude-standard", "-z"], cwd=repo_root)
    raw = result.stdout
    if not raw:
        return []
    names = [name for name in raw.split("\x00") if name]
    return sorted(names)


def _resolve_branch(repo_root: Path, branch_name: str) -> str:
    result = _run_git(["rev-parse", f"{branch_name}^{{commit}}"], cwd=repo_root, check=False)
    if result.returncode != 0:
        raise SourceResolutionError(
            f"Branch '{branch_name}' not found in repository {repo_root}"
        )
    return result.stdout.strip()


def _resolve_rev(repo_root: Path, revision: str) -> str:
    result = _run_git(["rev-parse", f"{revision}^{{commit}}"], cwd=repo_root, check=False)
    if result.returncode != 0:
        raise SourceResolutionError(
            f"Revision '{revision}' could not be resolved in repository {repo_root}"
        )
    return result.stdout.strip()


def resolve_source(source_spec: str, cwd: str | Path | None = None) -> ResolvedSource:
    """Resolve workspace/branch/rev source selection to concrete metadata."""
    workspace_root = Path(cwd or os.getcwd()).resolve()
    repo_root = _repo_root(workspace_root)
    git_available = repo_root is not None

    if source_spec == "workspace":
        if not git_available:
            return ResolvedSource(
                requested=source_spec,
                kind="workspace",
                workspace_root=workspace_root,
                repo_root=None,
                git_available=False,
                base_revision=None,
                branch_name=None,
                patch_text="",
                untracked_files=[],
            )
        assert repo_root is not None
        head = _head_revision(repo_root)
        return ResolvedSource(
            requested=source_spec,
            kind="workspace",
            workspace_root=workspace_root,
            repo_root=repo_root,
            git_available=True,
            base_revision=head,
            branch_name=None,
            patch_text=_workspace_patch(repo_root, head),
            untracked_files=_untracked_files(repo_root),
        )

    if not git_available or repo_root is None:
        raise SourceResolutionError(
            f"Source '{source_spec}' requires a git repository; none found at {workspace_root}"
        )

    if source_spec.startswith("branch:"):
        branch_name = source_spec.split(":", 1)[1].strip()
        if not branch_name:
            raise SourceResolutionError("branch source must include a branch name")
        return ResolvedSource(
            requested=source_spec,
            kind="branch",
            workspace_root=workspace_root,
            repo_root=repo_root,
            git_available=True,
            base_revision=_resolve_branch(repo_root, branch_name),
            branch_name=branch_name,
            patch_text="",
            untracked_files=[],
        )

    if source_spec.startswith("rev:"):
        revision = source_spec.split(":", 1)[1].strip()
        if not revision:
            raise SourceResolutionError("rev source must include a revision")
        return ResolvedSource(
            requested=source_spec,
            kind="rev",
            workspace_root=workspace_root,
            repo_root=repo_root,
            git_available=True,
            base_revision=_resolve_rev(repo_root, revision),
            branch_name=None,
            patch_text="",
            untracked_files=[],
        )

    raise SourceResolutionError(
        "Invalid source value. Expected one of: workspace, branch:<name>, rev:<sha>"
    )


def copy_untracked_files(repo_root: Path, destination_root: Path, relative_paths: list[str]) -> None:
    """Copy untracked files from repo root into destination root."""
    for rel_path in relative_paths:
        source_path = (repo_root / rel_path).resolve()
        destination_path = (destination_root / rel_path).resolve()
        if not source_path.exists():
            continue
        destination_path.parent.mkdir(parents=True, exist_ok=True)
        if source_path.is_dir():
            if destination_path.exists():
                shutil.rmtree(destination_path)
            shutil.copytree(source_path, destination_path)
        else:
            shutil.copy2(source_path, destination_path)
