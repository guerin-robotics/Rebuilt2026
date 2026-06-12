from __future__ import annotations

import importlib.util
from pathlib import Path


def _load_validator_module():
    script_path = (
        Path(__file__).resolve().parents[1]
        / "src/wpilib_agent_tools/integrations/codex/skill_bundle/scripts/validate_robot_repo.py"
    )
    spec = importlib.util.spec_from_file_location("validate_robot_repo", script_path)
    module = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(module)
    return module


def test_2026_profile_defaults_match_current_comp_dev_shape() -> None:
    module = _load_validator_module()

    defaults = module._normalize_profile_defaults("2026-robot-code")

    assert defaults["auto_path"] == "straight"
    assert defaults["state_key"] == "/AdvantageKit/RealOutputs/SwerveDrive/currentSystemState"
    assert defaults["expected_states"] == ["FOLLOW_PATH", "IDLE"]
    assert defaults["check_ds"] is True
    assert defaults["apply_profile_patch"] is True


def test_2026_profile_patch_uses_current_autos_api(tmp_path: Path) -> None:
    module = _load_validator_module()
    sandbox = tmp_path
    robot_dir = sandbox / "src/main/java/frc/robot"
    constants_dir = robot_dir / "constants"
    robot_dir.mkdir(parents=True)
    constants_dir.mkdir(parents=True)

    (robot_dir / "RobotContainer.java").write_text(
        """
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.autos.Autos;

public class RobotContainer {
    public Command getAutonomousCommand() {
        Command selectedAuto = autoChooser.getSelected();
        return selectedAuto != null ? selectedAuto : Commands.none();
    }
}
""".strip()
        + "\n",
        encoding="utf-8",
    )
    (robot_dir / "Robot.java").write_text(
        """
package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.constants.Constants;

public class Robot {
    public void robotInit() {
        m_robotContainer = RobotContainer.getInstance();
    }
}
""".strip()
        + "\n",
        encoding="utf-8",
    )
    (constants_dir / "Constants.java").write_text(
        """
package frc.robot.constants;

public class Constants {
    public enum Mode { REAL, SIM }
    public static final Mode currentMode = Mode.REAL;
}
""".strip()
        + "\n",
        encoding="utf-8",
    )
    (sandbox / "settings.gradle").write_text(
        """
pluginManagement {
}

if (file("../BLine-Lib").exists()) {
    includeBuild("../BLine-Lib")
}
""".strip()
        + "\n",
        encoding="utf-8",
    )

    notes = module._patch_2026_profile(sandbox, "straight")

    robot_container = (robot_dir / "RobotContainer.java").read_text(encoding="utf-8")
    robot_java = (robot_dir / "Robot.java").read_text(encoding="utf-8")
    constants_java = (constants_dir / "Constants.java").read_text(encoding="utf-8")

    assert 'return Autos.followPath("straight", true);' in robot_container
    assert "DriverStationSim.setAutonomous(true);" in robot_java
    assert "public static final Mode currentMode = Mode.SIM;" in constants_java
    assert "set autonomous command to Autos.followPath(auto_path, true)" in notes
