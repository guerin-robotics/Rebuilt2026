from __future__ import annotations

import importlib.util
from pathlib import Path


def _load_module():
    script_path = Path(__file__).resolve().parents[2] / "scripts" / "validate_skill.py"
    spec = importlib.util.spec_from_file_location("validate_skill_script", script_path)
    module = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(module)
    return module


def test_validate_skill_accepts_minimal_valid_skill(tmp_path: Path) -> None:
    module = _load_module()
    skill_dir = tmp_path / "skill"
    (skill_dir / "agents").mkdir(parents=True)
    (skill_dir / "SKILL.md").write_text(
        "---\nname: sample-skill\ndescription: This is a sufficiently long description for validation.\n---\n\n# Sample\n",
        encoding="utf-8",
    )
    (skill_dir / "agents" / "openai.yaml").write_text(
        "interface:\n  display_name: Sample Skill\n  short_description: Useful short description\n  default_prompt: Use $sample-skill for this task.\n",
        encoding="utf-8",
    )

    assert module.validate_skill(skill_dir) == []


def test_validate_skill_reports_missing_prompt_reference(tmp_path: Path) -> None:
    module = _load_module()
    skill_dir = tmp_path / "skill"
    (skill_dir / "agents").mkdir(parents=True)
    (skill_dir / "SKILL.md").write_text(
        "---\nname: sample-skill\ndescription: This is a sufficiently long description for validation.\n---\n",
        encoding="utf-8",
    )
    (skill_dir / "agents" / "openai.yaml").write_text(
        "interface:\n  display_name: Sample Skill\n  short_description: Useful short description\n  default_prompt: Use another skill for this task.\n",
        encoding="utf-8",
    )

    errors = module.validate_skill(skill_dir)
    assert any("default_prompt must reference $<skill-name>" in err for err in errors)
