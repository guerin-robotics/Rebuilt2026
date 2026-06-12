#!/usr/bin/env python3
"""Local validator for Codex skill folders."""

from __future__ import annotations

import re
import sys
from pathlib import Path


def _parse_simple_yaml_mapping(text: str) -> dict[str, object]:
    root: dict[str, object] = {}
    stack: list[tuple[int, dict[str, object]]] = [(-1, root)]

    for raw_line in text.splitlines():
        if not raw_line.strip() or raw_line.lstrip().startswith("#"):
            continue

        indent = len(raw_line) - len(raw_line.lstrip(" "))
        if indent % 2 != 0:
            raise ValueError("YAML indentation must use multiples of two spaces")

        line = raw_line.strip()
        if ":" not in line:
            raise ValueError(f"Invalid YAML line: {raw_line}")
        key, value = line.split(":", 1)
        key = key.strip()
        value = value.strip()
        if not key:
            raise ValueError(f"Invalid YAML key in line: {raw_line}")

        while stack and indent <= stack[-1][0]:
            stack.pop()
        if not stack:
            raise ValueError(f"Could not determine YAML parent for line: {raw_line}")
        parent = stack[-1][1]

        if value:
            if value[0] == value[-1] and value[0] in {"'", '"'}:
                value = value[1:-1]
            parent[key] = value
            continue

        child: dict[str, object] = {}
        parent[key] = child
        stack.append((indent, child))

    return root


def _frontmatter(skill_md: Path) -> dict[str, object]:
    content = skill_md.read_text(encoding="utf-8")
    match = re.match(r"^---\n(.*?)\n---", content, re.DOTALL)
    if not match:
        raise ValueError("SKILL.md frontmatter missing or malformed")
    parsed = _parse_simple_yaml_mapping(match.group(1))
    if not isinstance(parsed, dict):
        raise ValueError("SKILL.md frontmatter must be a YAML dictionary")
    return parsed


def validate_skill(skill_dir: Path) -> list[str]:
    errors: list[str] = []
    skill_md = skill_dir / "SKILL.md"
    openai_yaml = skill_dir / "agents" / "openai.yaml"

    if not skill_md.exists():
        return [f"missing file: {skill_md}"]
    if not openai_yaml.exists():
        errors.append(f"missing file: {openai_yaml}")

    try:
        fm = _frontmatter(skill_md)
    except Exception as exc:
        return [str(exc)]

    allowed = {"name", "description"}
    extra = set(fm.keys()) - allowed
    if extra:
        errors.append(f"unexpected frontmatter keys: {sorted(extra)}")

    name = fm.get("name")
    desc = fm.get("description")

    if not isinstance(name, str) or not re.fullmatch(r"[a-z0-9-]{1,64}", name):
        errors.append("frontmatter.name must be hyphen-case (1-64 chars)")
    if not isinstance(desc, str) or len(desc.strip()) < 20:
        errors.append("frontmatter.description must be a non-trivial string")

    if openai_yaml.exists():
        try:
            data = _parse_simple_yaml_mapping(openai_yaml.read_text(encoding="utf-8"))
        except Exception as exc:
            errors.append(f"invalid YAML in agents/openai.yaml: {exc}")
            data = None

        if isinstance(data, dict):
            interface = data.get("interface")
            if not isinstance(interface, dict):
                errors.append("agents/openai.yaml missing interface mapping")
            else:
                for key in ("display_name", "short_description", "default_prompt"):
                    if not isinstance(interface.get(key), str) or not interface.get(key).strip():
                        errors.append(f"agents/openai.yaml interface.{key} is required")
                prompt = str(interface.get("default_prompt", ""))
                if isinstance(name, str) and f"${name}" not in prompt:
                    errors.append("interface.default_prompt must reference $<skill-name>")

    return errors


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: python scripts/validate_skill.py <skill_dir>")
        return 2

    skill_dir = Path(sys.argv[1]).expanduser().resolve()
    errors = validate_skill(skill_dir)
    if errors:
        print("Skill validation failed:")
        for error in errors:
            print(f"- {error}")
        return 1

    print(f"Skill is valid: {skill_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
