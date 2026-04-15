// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class HubShiftUtil {
  /**
   * Utility class used to determine which "hub shift" (active/inactive windows)
   * are in effect during a match.
   *
   * This file contains constants for scheduled periods during teleop, helper
   * methods for converting DriverStation/FMS inputs into which alliance is
   * "active first", and convenience Commands for toggling/testing the utility.
   *
   * Comments are added to help students understand each value and method.
   */
  public enum ShiftEnum {
    TRANSITION,
    SHIFT1,
    SHIFT2,
    SHIFT3,
    SHIFT4,
    ENDGAME,
    AUTO,
    DISABLED;
  }

  // When true, flips which alliance is considered the initial "winning" alliance
  // for testing purposes. This is used to override the alliance selection logic
  // (see `getFirstActiveAlliance`). Default false (no flip).
  public static boolean flipped = false;

  // When true, disables hub-shift logic entirely. Used to temporarily ignore
  // the schedule and treat all times as inactive for testing or disabling.
  public static boolean disabled = false;

  /**
   * Simple immutable struct that describes the currently applicable shift.
   *
   * Fields:
   * - currentShift: which ShiftEnum period we're in (AUTO, SHIFT1.., ENDGAME...)
   * - elapsedTime: how many seconds have elapsed since the start of the
   *   current shift window (combined windows considered when appropriate)
   * - remainingTime: how many seconds remain until the end of the current
   *   shift window (combined windows considered when appropriate)
   * - active: whether this shift is an "active" hub period (true) or an
   *   "inactive" period (false)
   */
  public record ShiftInfo(
    ShiftEnum currentShift, double elapsedTime, double remainingTime, boolean active) {}

  // Internal timer used to track elapsed match time for hub shift calculations.
  // This timer is started/restarted by `initialize()` at the beginning of
  // teleop so that the utility can compute relative times.
  private static Timer shiftTimer = new Timer();

  // Cached array of enum values so we can index into them using an index
  // derived from the start/end time arrays.
  private static final ShiftEnum[] shiftsEnums = ShiftEnum.values();

  // Official (baseline) teleop schedule windows. These arrays define the
  // start and end seconds (relative to teleop start) of the named shifts.
  // Index 0 -> SHIFT1 (0-10), index 1 -> SHIFT2 (10-35), etc.; the last window
  // ends at 140 which is the full teleop duration used for indexing purposes.
  private static final double[] shiftStartTimes = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
  private static final double[] shiftEndTimes = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};

  // Timing tolerances and fudge factors used to create the "shifted" schedule
  // (getShiftedShiftInfo). These attempt to account for ball time-of-flight and
  // sensor counting delays so that the robot can extend or advance active
  // windows slightly to avoid cutting off shots or counts.
  // Typical units are seconds.
  private static final double minFuelCountDelay = 1.0; // minimum delay for fuel counting
  private static final double maxFuelCountDelay = 2.0; // maximum delay for fuel counting
  // When a shift ends, extend the active window by this amount to allow
  // in-flight actions to complete (e.g., a ball already in the air).
  private static final double shiftEndFuelCountExtension = 3.0;
  // Minimum/maximum time-of-flight estimates for a fired ball (seconds).
  private static final double minTimeOfFlight = 2.5; // ShotCalculator placeholder
  private static final double maxTimeOfFlight = 3.0; // ShotCalculator placeholder
  // Fudge values applied to start/end times when calculating shifted windows.
  // approachingActiveFudge moves an active window earlier to account for the
  // ball's time of flight plus minimum counting delay.
  private static final double approachingActiveFudge = -1 * (minTimeOfFlight + minFuelCountDelay);
  // endingActiveFudge extends end windows to account for in-flight balls and
  // maximum counting delay, plus any explicit end extension.
  private static final double endingActiveFudge =
    shiftEndFuelCountExtension + -1 * (maxTimeOfFlight + maxFuelCountDelay);

  // Tracks whether the first-active alliance is flipped for testing. Not
  // currently used directly (kept for compatibility with older code paths).
  private static boolean firstActiveAlliance = false;
  // End of autonomous period (seconds) - used when in AUTO mode
  public static final double autoEndTime = 20.0;
  // Total teleop duration used for scheduling math (seconds)
  public static final double teleopDuration = 140.0;
  // Default schedule arrays indicating whether each scheduled window is
  // considered "active" (true) or "inactive" (false) for the alliance that
  // starts active.
  private static final boolean[] activeSchedule = {true, true, false, true, false, true};
  private static final boolean[] inactiveSchedule = {true, false, true, false, true, true};
  // If the local timer and the field-supplied teleop clock differ by this
  // many seconds or more, adjust the internal offset to re-sync (used when
  // FMS is attached and teleop time is reported by DriverStation).
  private static final double timeResetThreshold = 3.0;
  // Offset applied to the internal shiftTimer to keep it aligned with the
  // field teleop clock when running on an actual field with FMS attached.
  private static double shiftTimerOffset = 0.0;
  // Optional override supplier that can be set for tests to force which
  // alliance is treated as the initial "winning" alliance. When present,
  // the supplier should return Optional.of(true/false) to indicate flip.
  @Setter private static Supplier<Optional<Boolean>> allianceWinOverride = () -> Optional.empty();

  public static Optional<Boolean> getAllianceWinOverride() {
    return allianceWinOverride.get();
  }

  /**
   * Returns the optional override that may force which alliance is considered
   * the "first active". This is useful for testing and flipping behavior
   * without relying on FMS messages.
   */

  public static Boolean isActiveFirst() {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    // Returns true if the current DriverStation alliance is the one that is
    // considered the first active alliance according to `getFirstActiveAlliance()`.
    return alliance == getFirstActiveAlliance();
  }

  public static Alliance getFirstActiveAlliance() {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    // 1) If an explicit override is provided, use it. The override is a
    //    boolean where `true` means we flip the original alliance (use the
    //    opposite), and `false` means keep it as-is. The logic below returns
    //    which alliance should be used as the one that is "first active".
    var winOverride = getAllianceWinOverride();
    if (!winOverride.isEmpty()) {
      return winOverride.get()
          ? (alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue)
          : (alliance == Alliance.Blue ? Alliance.Blue : Alliance.Red);
    }

    // 2) If there is an FMS Game Specific Message (GSM), interpret its first
    //    character. Historically, this message encoded which side (R/B)
    //    starts with the control of the hub. We invert intentionally because
    //    of historical mapping between message characters and field sides.
    String message = DriverStation.getGameSpecificMessage();
    if (message.length() > 0) {
      char character = message.charAt(0);
      if (character == 'R') {
        return Alliance.Blue;
      } else if (character == 'B') {
        return Alliance.Red;
      }
    }

    // 3) If nothing else available, return the opposite alliance as the
    //    default "first active" to provide a deterministic fallback.
    return alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
  }

  /** Starts the timer at the begining of teleop. */
  public static void initialize() {
    // Reset any offset adjustments and restart the internal timer. Call at
    // the start of teleop so relative times are computed from zero.
    shiftTimerOffset = 0;
    shiftTimer.restart();
  }

  private static boolean[] getSchedule() {
    boolean[] currentSchedule;
    Alliance startAlliance = getFirstActiveAlliance();
    currentSchedule =
        startAlliance == DriverStation.getAlliance().orElse(Alliance.Blue)
            ? activeSchedule
            : inactiveSchedule;
    return currentSchedule;
  }

  /**
   * Returns the schedule to use based on which alliance is considered to
   * start active. If the alliance that the DriverStation reports matches the
   * alliance that `getFirstActiveAlliance()` returned, we use the
   * `activeSchedule`, otherwise we use `inactiveSchedule`.
   */

  private static ShiftInfo getShiftInfo(
      boolean[] currentSchedule, double[] shiftStartTimes, double[] shiftEndTimes) {
    double timerValue = shiftTimer.get();
    double currentTime = timerValue - shiftTimerOffset;
    double stateTimeElapsed = currentTime;
    double stateTimeRemaining = 0.0;
    boolean active = false;
    ShiftEnum currentShift = ShiftEnum.DISABLED;
    double fieldTeleopTime = 140.0 - DriverStation.getMatchTime();

    if (DriverStation.isAutonomousEnabled()) {
      stateTimeElapsed = currentTime;
      stateTimeRemaining = autoEndTime - currentTime;
      active = true;
      currentShift = ShiftEnum.AUTO;
    } else if (DriverStation.isEnabled()) {
      // Adjust the current offset if the time difference above the theshold
      if (Math.abs(fieldTeleopTime - currentTime) >= timeResetThreshold
          && fieldTeleopTime <= 135
          && DriverStation.isFMSAttached()) {
        shiftTimerOffset += currentTime - fieldTeleopTime;
        currentTime = timerValue + shiftTimerOffset;
      }
      int currentShiftIndex = -1;
      for (int i = 0; i < shiftStartTimes.length; i++) {
        if (currentTime >= shiftStartTimes[i] && currentTime < shiftEndTimes[i]) {
          currentShiftIndex = i;
          break;
        }
      }
      if (currentShiftIndex < 0) {
        // After last shift, so assume endgame
        currentShiftIndex = shiftStartTimes.length - 1;
      }

      // Calculate elapsed and remaining time in the current shift, ignoring combined shifts
      stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex];
      stateTimeRemaining = shiftEndTimes[currentShiftIndex] - currentTime;

      // If the state is the same as the last shift, combine the elapsed time
      if (currentShiftIndex > 0) {
        if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex - 1]) {
          stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex - 1];
        }
      }

      // If the state is the same as the next shift, combine the remaining time
      if (currentShiftIndex < shiftEndTimes.length - 1) {
        if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex + 1]) {
          stateTimeRemaining = shiftEndTimes[currentShiftIndex + 1] - currentTime;
        }
      }

      active = currentSchedule[currentShiftIndex];
      currentShift = shiftsEnums[currentShiftIndex];
    }
    ShiftInfo shiftInfo = new ShiftInfo(currentShift, stateTimeElapsed, stateTimeRemaining, active);
    return shiftInfo;
  }

  /**
   * Core method that computes which shift window we're currently in and the
   * elapsed/remaining times for it. The method takes a schedule and arrays of
   * start/end times so it can be used for both the official schedule and a
   * "shifted" schedule that applies fudge factors.
   *
   * Behavior notes:
   * - If the Robot is in Autonomous, returns AUTO with remaining time until
   *   `autoEndTime`.
   * - If enabled in teleop, finds the matching index into the provided
   *   start/end arrays and computes elapsed/remaining times. If adjacent
   *   windows have the same active/inactive value, the elapsed/remaining
   *   times are combined across the boundary to present a single continuous
   *   active/inactive period.
   * - If a significant discrepancy exists between the local timer and the
   *   FMS teleop clock, the offset is adjusted to re-sync (when FMS attached).
   */

  public static ShiftInfo getOfficialShiftInfo() {
    return getShiftInfo(getSchedule(), shiftStartTimes, shiftEndTimes);
  }

  /**
   * Returns shift information computed from the official (baseline) schedule
   * defined by `shiftStartTimes`/`shiftEndTimes` and the current schedule
   * (active/inactive) for the match.
   */

  public static ShiftInfo getShiftedShiftInfo() {
    boolean[] shiftSchedule = getSchedule();
    // Starting active
    if (shiftSchedule[1] == true) {
      double[] shiftedShiftStartTimes = {
        0.0,
        10.0,
        35.0 + endingActiveFudge,
        60.0 + approachingActiveFudge,
        85.0 + endingActiveFudge,
        110.0 + approachingActiveFudge
      };
      double[] shiftedShiftEndTimes = {
        10.0,
        35.0 + endingActiveFudge,
        60.0 + approachingActiveFudge,
        85.0 + endingActiveFudge,
        110.0 + approachingActiveFudge,
        140.0
      };
      return getShiftInfo(shiftSchedule, shiftedShiftStartTimes, shiftedShiftEndTimes);
    }
    double[] shiftedShiftStartTimes = {
      0.0,
      10.0 + endingActiveFudge,
      35.0 + approachingActiveFudge,
      60.0 + endingActiveFudge,
      85.0 + approachingActiveFudge,
      110.0
    };
    double[] shiftedShiftEndTimes = {
      10.0 + endingActiveFudge,
      35.0 + approachingActiveFudge,
      60.0 + endingActiveFudge,
      85.0 + approachingActiveFudge,
      110.0,
      140.0
    };
    return getShiftInfo(shiftSchedule, shiftedShiftStartTimes, shiftedShiftEndTimes);
    // }
  }

  /**
   * Returns a shifted version of the shift info which applies the precomputed
   * fudge factors (approachingActiveFudge / endingActiveFudge) to start/end
   * times. The goal is to slightly advance or extend active windows to
   * account for ball flight time and sensor delays so the robot doesn't
   * prematurely cut off scoring actions.
   */

  public static void changeFlipped() {
    if (flipped) {
      flipped = false;
      HubShiftUtil.setAllianceWinOverride(() -> Optional.of(HubShiftUtil.flipped));
    } else {
      flipped = true;
      HubShiftUtil.setAllianceWinOverride(() -> Optional.of(HubShiftUtil.flipped));
    }
  }

  /**
   * Toggle the `flipped` test flag and set the `allianceWinOverride` so the
   * rest of this utility will treat the alliance as flipped. Useful for
   * operator testing when you want to pretend the other alliance started
   * active.
   */

  public static Command flipWinner() {
    return new InstantCommand(() -> changeFlipped());
  }

  /**
   * Returns a simple InstantCommand that flips which alliance is considered
   * first-active. Can be bound to a button for quick operator testing.
   */

  public static void toggleDisable() {
    disabled = !disabled;
    Logger.recordOutput("RobotState/disabled", disabled);
  }

  /**
   * Toggle the disabled flag for this utility and record it to the logger.
   * When disabled is true, other code can choose to ignore hub shift state.
   */

  public static Command disableHubShiftUtil() {
    return new InstantCommand(() -> toggleDisable());
  }
}
