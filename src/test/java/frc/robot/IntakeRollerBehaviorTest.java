package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.AllianceFlipUtil;
import frc.robot.subsystems.drive.Drive;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Simulation harness for the intake roller duty cycle.
 *
 * <p>Drives the real {@link RobotContainer} bindings with a simulated DriverStation and a stepped
 * FPGA clock, then reads the voltage actually commanded to the roller leader TalonFX each 20 ms
 * tick. Nothing here re-implements robot logic — the assertions observe the shipping bindings.
 *
 * <p>Expected duty cycle (the behavior these tests are written against):
 *
 * <ol>
 *   <li>Idle: roller runs continuously at {@code intakeRollerVoltage}.
 *   <li>Shoot/pass requested: roller drops to 0 V.
 *   <li>The moment the feeders and transport are allowed to run: roller goes to {@code
 *       intakeRollerAgitateVoltage}.
 *   <li>Shoot/pass released: roller returns to {@code intakeRollerVoltage}.
 * </ol>
 */
public class IntakeRollerBehaviorTest {

  private static final double INTAKE_V =
      HardwareConstants.CompConstants.Voltages.intakeRollerVoltage.magnitude();
  private static final double AGITATE_V =
      HardwareConstants.CompConstants.Voltages.intakeRollerAgitateVoltage.magnitude();
  private static final double EPS = 1e-9;

  private static final int SHOOT_BUTTON = 1; // Thrustmaster button 1 -> Triggers.shootButton()
  private static final int JOYSTICK_PORT =
      HardwareConstants.ControllerConstants.JoystickControllerPort;

  private static RobotContainer container;
  private static Drive drive;
  private static TalonFX rollerLeader;
  private static TalonFX lowerFeederMotor;

  private static Pose2d alignedPose;
  private static Pose2d misalignedPose;
  private static Pose2d heldPose;

  // ── reflection helpers ──────────────────────────────────────────────────────

  private static Object get(Object target, String name) throws Exception {
    Class<?> c = target.getClass();
    while (c != null) {
      try {
        Field f = c.getDeclaredField(name);
        f.setAccessible(true);
        return f.get(target);
      } catch (NoSuchFieldException e) {
        c = c.getSuperclass();
      }
    }
    throw new NoSuchFieldException(name + " on " + target.getClass());
  }

  /** Finds the first TalonFX-typed field on a subsystem's IO object. */
  private static TalonFX firstTalon(Object subsystem) throws Exception {
    Object io = get(subsystem, "io");
    for (Field f : io.getClass().getDeclaredFields()) {
      if (TalonFX.class.isAssignableFrom(f.getType())) {
        f.setAccessible(true);
        return (TalonFX) f.get(io);
      }
    }
    throw new NoSuchFieldException("no TalonFX on " + io.getClass());
  }

  // ── harness ─────────────────────────────────────────────────────────────────

  @BeforeAll
  static void setUpAll() throws Exception {
    assert HAL.initialize(500, 0);
    SimHooks.pauseTiming();

    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setTest(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setJoystickButtonCount(JOYSTICK_PORT, 12);
    DriverStationSim.setJoystickAxisCount(JOYSTICK_PORT, 4);
    DriverStationSim.setJoystickPOVCount(JOYSTICK_PORT, 1);
    DriverStationSim.notifyNewData();

    HardwareConstants.ControllerConstants.XBOX_DRIVE_MODE = false;
    HardwareConstants.TuningConstants.TUNING_MODE = false;
    HardwareConstants.TuningConstants.DEMO_MODE = false;
    // Take the hub-shift schedule out of the picture so isShootSafeTime is always true.
    frc.robot.util.HubShiftUtil.disabled = true;

    CommandScheduler.getInstance().cancelAll();
    container = new RobotContainer();

    // The constructor installs configureSimBindings() because Robot.isSimulation() is true in a
    // desktop test. Add the real robot bindings on top: they are driven from the Thrustmaster
    // (port 2), while the sim bindings read the sim keyboard (port 3) and Xbox (port 1), neither
    // of which this test touches. Without this, we'd only be testing the sim bindings.
    Method m = RobotContainer.class.getDeclaredMethod("configureStateBindings");
    m.setAccessible(true);
    m.invoke(container);

    drive = (Drive) get(container, "drive");
    rollerLeader = firstTalon(get(container, "intakeRoller"));
    lowerFeederMotor = firstTalon(get(container, "lowerFeeder"));

    AllianceFlipUtil.refresh();

    // Park the robot inside our own alliance zone and compute the exact hub-facing heading.
    drive.setPose(new Pose2d(2.0, 4.0, Rotation2d.kZero));
    Rotation2d toHub = RobotState.getInstance().getAngleToAllianceHub();
    alignedPose = new Pose2d(2.0, 4.0, toHub);
    misalignedPose = new Pose2d(2.0, 4.0, toHub.plus(Rotation2d.fromDegrees(90)));
    heldPose = alignedPose;

    assertEquals(
        HardwareConstants.Zones.broadZone.ALLIANCE_ZONE,
        RobotState.getInstance().getBroadZone(),
        "test pose must be inside our alliance zone for the hub-shot bindings to arm");
  }

  @BeforeEach
  void resetBetweenTests() {
    for (int b = 1; b <= 12; b++) {
      DriverStationSim.setJoystickButton(JOYSTICK_PORT, b, false);
    }
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();

    CommandScheduler.getInstance().cancelAll();
    heldPose = alignedPose;

    // Park every shooter mechanism, then let the flywheel settle back to its idle speed so the
    // next shot has a real spin-up phase to wait through.
    container.getAutoStopCommand().schedule();
    tick(60);
    assertEquals(INTAKE_V, rollerVolts(), EPS, "roller must idle at full intake voltage");
  }

  /** Runs one 20 ms robot loop with the pose held and DriverStation data refreshed. */
  private static void tick() {
    drive.setPose(heldPose);
    DriverStationSim.notifyNewData();
    AllianceFlipUtil.refresh();
    SimHooks.stepTiming(0.02);
    CommandScheduler.getInstance().run();
  }

  private static void tick(int n) {
    for (int i = 0; i < n; i++) {
      tick();
    }
  }

  /** Voltage currently commanded to the roller leader. A zero-velocity request reads as 0 V. */
  private static double rollerVolts() {
    ControlRequest r = rollerLeader.getAppliedControl();
    if (r instanceof VoltageOut v) {
      return v.Output;
    }
    String s = r.getControlInfo().get("Velocity");
    if (s != null && Double.parseDouble(s) == 0.0) {
      return 0.0;
    }
    throw new IllegalStateException("unexpected roller control request: " + r.getControlInfo());
  }

  /** True once the lower feeder is being commanded to a non-zero velocity. */
  private static boolean feedersRunning() {
    String s = lowerFeederMotor.getAppliedControl().getControlInfo().get("Velocity");
    return s != null && Double.parseDouble(s) != 0.0;
  }

  private static void press(int button, boolean pressed) {
    DriverStationSim.setJoystickButton(JOYSTICK_PORT, button, pressed);
    DriverStationSim.notifyNewData();
  }

  /**
   * Runs {@code loops} ticks and checks the roller against the feeder gate: 0 V before the feeders
   * start, agitate voltage from the tick they start onward.
   */
  private static void assertRollerTracksFeederGate(int loops, String label) {
    int feederStart = -1;
    for (int i = 0; i < loops; i++) {
      tick();
      boolean feeding = feedersRunning();
      if (feederStart < 0 && feeding) {
        feederStart = i;
      }
      double v = rollerVolts();
      if (feederStart < 0) {
        assertEquals(
            0.0,
            v,
            EPS,
            label + ": tick " + i + " — feeders not gated on yet, roller must be at 0 V");
      } else {
        assertEquals(
            AGITATE_V,
            v,
            EPS,
            label
                + ": tick "
                + i
                + " — feeders started at tick "
                + feederStart
                + ", roller must be at agitate voltage");
      }
    }
    assertTrue(feederStart >= 0, label + ": feeders never started within " + loops + " ticks");
    assertTrue(
        feederStart > 0,
        label
            + ": expected a non-zero interval where the roller is held at 0 V before the feeders"
            + " are gated on, but the feeders started on the very first tick");
  }

  // ── autonomous ──────────────────────────────────────────────────────────────

  @Test
  void auto_rollerRunsContinuouslyWhenNotShooting() {
    DriverStationSim.setAutonomous(true);
    tick(25);
    assertEquals(INTAKE_V, rollerVolts(), EPS, "auto idle: roller must run at intake voltage");
  }

  @Test
  void auto_shootHoldsRollerAtZeroUntilFeedersStartThenAgitates() {
    DriverStationSim.setAutonomous(true);
    Command shoot = NamedCommands.getCommand("Shoot");
    shoot.schedule();
    assertRollerTracksFeederGate(60, "auto shoot");
    shoot.cancel();
  }

  @Test
  void auto_rollerReturnsToIntakeVoltageAfterShoot() {
    DriverStationSim.setAutonomous(true);
    Command shoot = NamedCommands.getCommand("Shoot");
    shoot.schedule();
    tick(45);
    assertEquals(AGITATE_V, rollerVolts(), EPS, "auto shoot should be agitating by now");

    shoot.cancel();
    NamedCommands.getCommand("stopAll").schedule();
    tick(10);
    assertEquals(
        INTAKE_V, rollerVolts(), EPS, "after the auto shot the roller must run continuously again");
  }

  // ── teleop ──────────────────────────────────────────────────────────────────

  @Test
  void teleop_rollerRunsContinuouslyWhenNotShooting() {
    tick(25);
    assertEquals(INTAKE_V, rollerVolts(), EPS, "teleop idle: roller must run at intake voltage");
  }

  @Test
  void teleop_shootHoldsRollerAtZeroUntilFeedersStartThenAgitates() {
    press(SHOOT_BUTTON, true);
    assertRollerTracksFeederGate(60, "teleop shoot");
    press(SHOOT_BUTTON, false);
  }

  @Test
  void teleop_rollerReturnsToIntakeVoltageWhenShootReleased() {
    press(SHOOT_BUTTON, true);
    tick(45);
    assertEquals(AGITATE_V, rollerVolts(), EPS, "teleop shoot should be agitating by now");

    press(SHOOT_BUTTON, false);
    tick(10);
    assertEquals(
        INTAKE_V,
        rollerVolts(),
        EPS,
        "once the shoot button is released the roller must run continuously again");
  }

  @Test
  void teleop_rollerDoesNotResumeFullIntakeWhenAlignmentIsLostMidShot() {
    press(SHOOT_BUTTON, true);
    tick(45);
    assertEquals(AGITATE_V, rollerVolts(), EPS, "teleop shoot should be agitating by now");

    // Driver is still holding shoot, but the robot swings off target.
    heldPose = misalignedPose;
    for (int i = 0; i < 20; i++) {
      tick();
      assertNotEquals(
          INTAKE_V,
          rollerVolts(),
          EPS,
          "tick "
              + i
              + ": losing alignment mid-shot must not put the roller back to full intake voltage"
              + " while the shoot button is still held");
    }
    press(SHOOT_BUTTON, false);
  }
}
