// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  // CHANGE ME TO WHAT ROBOT IS CURRENTLY BEING USED
  public static final RobotType robotType = RobotType.COMP;

  /**
   * Selects the simulation backend when {@link #currentMode} is {@link Mode#SIM}.
   *
   * <p>{@code true} — MapleSim rigid-body physics: real mass, wheel traction limits, field
   * collisions, and 2026 Rebuilt Fuel that can be intaked and shot.
   *
   * <p>{@code false} — the original per-module {@code DCMotorSim} path ({@code ModuleIOSim}), which
   * has no notion of mass or collisions. Kept as a fallback because maple-sim 0.4.0 is a beta.
   *
   * <p>Has no effect on a real robot: {@link #currentMode} is {@link Mode#REAL} there regardless.
   */
  public static final boolean useMapleSim = true;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum RobotType {
    COMP,
    ALPHA,
    NONE
  }

  public static boolean disableHAL = false;
}
