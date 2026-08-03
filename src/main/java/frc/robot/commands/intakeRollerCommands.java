package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.intakeRoller.intakeRoller;
import frc.robot.subsystems.upperFeeder.UpperFeeder;
import java.util.function.BooleanSupplier;

public class intakeRollerCommands {

  public static Command setRollerVoltage(intakeRoller intakeRoller, Voltage voltage) {
    return Commands.startEnd(
            () -> intakeRoller.setRollerVoltage(voltage),
            () -> intakeRoller.setRollerVoltage(Volts.of(0)),
            intakeRoller)
        .withName("IntakeRollerVoltage_" + voltage.in(Volts) + "V");
  }

  public static Command setRollerVelocity(intakeRoller intakeRoller, AngularVelocity rollerVelo) {
    return Commands.runOnce(() -> intakeRoller.setRollerVelocity(rollerVelo), intakeRoller)
        .withName("IntakeRollerVelocity");
  }

  /**
   * Runs the roller at a closed-loop velocity for as long as this command runs, then zeroes it.
   *
   * <p>This is the velocity-control counterpart to {@link #setRollerVoltage} and exists because
   * {@link #setRollerVelocity} is a {@code runOnce} — it ends immediately, so it cannot hold the
   * subsystem as a default command. This one uses {@code startEnd} so it never ends on its own.
   *
   * <p>Stops by commanding 0 V rather than 0 RPM: the velocity request goes out as torque current,
   * so a 0 RPM setpoint would actively brake the roller against the hopper instead of coasting.
   *
   * @param intakeRoller The intake roller subsystem
   * @param rollerVelo The closed-loop velocity setpoint to hold
   */
  public static Command runRollerAtVelocity(intakeRoller intakeRoller, AngularVelocity rollerVelo) {
    return Commands.startEnd(
            () -> intakeRoller.setRollerVelocity(rollerVelo),
            () -> intakeRoller.setRollerVoltage(Volts.of(0)),
            intakeRoller)
        .withName("IntakeRoller_Velocity_" + (int) rollerVelo.in(RPM) + "RPM");
  }

  public static Command intakeRollerIdle(intakeRoller intakeRoller) {
    return Commands.run(
            () ->
                intakeRoller.setRollerVoltage(
                    HardwareConstants.CompConstants.Voltages.intakeRollerAgitateVoltage),
            intakeRoller)
        .withName("intakeRollerIdle");
  }

  public static Command stopIntakeRoller(intakeRoller intakeRoller) {
    return Commands.runOnce(
            () -> intakeRoller.setRollerVelocity(RotationsPerSecond.of(0)), intakeRoller)
        .withName("IntakeRollerStop");
  }

  /**
   * Holds the roller at zero for as long as this command runs. Unlike {@link #stopIntakeRoller},
   * this does not end, so it keeps ownership of the subsystem and the always-on default command
   * cannot re-engage until the command is interrupted or its button is released.
   *
   * @param intakeRoller The intake roller subsystem
   */
  public static Command holdRollerStopped(intakeRoller intakeRoller) {
    return Commands.startEnd(
            () -> intakeRoller.setRollerVoltage(Volts.of(0)),
            () -> intakeRoller.setRollerVoltage(Volts.of(0)),
            intakeRoller)
        .withName("IntakeRoller_HoldStopped");
  }

  /**
   * Runs the intake roller backwards for as long as this command runs, to back a jam out of the
   * intake. Like {@link #holdRollerStopped}, this requires the subsystem and does not end on its
   * own, so it interrupts whatever owns the roller — the always-on default or a shot's agitate —
   * and keeps ownership until its button is released. On release the roller is zeroed and the
   * always-on default re-engages at intake voltage on the next loop.
   *
   * @param intakeRoller The intake roller subsystem
   */
  public static Command reverseRoller(intakeRoller intakeRoller) {
    return Commands.startEnd(
            () ->
                intakeRoller.setRollerVoltage(
                    HardwareConstants.CompConstants.Voltages.intakeRollerReverseVoltage),
            () -> intakeRoller.setRollerVoltage(Volts.of(0)),
            intakeRoller)
        .withName("IntakeRoller_Reverse");
  }

  /**
   * Holds the roller at zero for {@code flywheelSpinupSeconds}, then agitates. This is the roller
   * half of the hard-coded tower shot: the tower feeders and transport use the no-align {@link
   * FeederCommands#setUpperVelocityAfterWait(UpperFeeder, AngularVelocity)} overloads, which wait
   * the same fixed spin-up time and then release unconditionally. Mirroring that wait here starts
   * the roller on the same loop the fuel starts moving.
   *
   * <p>Do not use this for the hub or pass shots — those gate their feed on alignment, so their
   * roller binding must gate on the same condition via {@link #setVoltageAfterWait}.
   *
   * @param intakeRoller The intake roller subsystem
   * @param voltage The agitate voltage to apply once the spin-up wait has elapsed
   */
  public static Command setVoltageAfterSpinupWait(intakeRoller intakeRoller, Voltage voltage) {
    return Commands.sequence(
            new WaitCommand(HardwareConstants.CompConstants.Waits.flywheelSpinupSeconds),
            setRollerVoltage(intakeRoller, voltage))
        .withName("IntakeRoller_AgitateAfterSpinup");
  }

  /**
   * Runs the intake roller at the given agitate voltage, but only once the shot is actually being
   * fed. The caller passes the same condition its feeder/transport binding gates on, so the roller
   * starts agitating on the same loop the fuel starts moving — agitating earlier just stirs the
   * hopper while nothing is being fed, and agitating later wastes part of the shot.
   *
   * <p>Unlike the feeder/transport wait sequences, this does not fall through to agitating after
   * the timeout — if the shot never becomes ready, the roller is held at zero (claiming the
   * subsystem so the default command cannot re-engage) until the caller's whileTrue binding is
   * released.
   *
   * <p>See {@link FeederCommands#setLowerVelocityAfterWait} for full details on the wait logic.
   *
   * @param intakeRoller The intake roller subsystem
   * @param voltage The agitate voltage to apply once ready
   * @param readyToFeed Supplier that returns true when the feeders and transport are allowed to run
   */
  public static Command setVoltageAfterWait(
      intakeRoller intakeRoller, Voltage voltage, BooleanSupplier readyToFeed) {
    return Commands.sequence(
            Commands.waitUntil(readyToFeed)
                .withTimeout(HardwareConstants.CompConstants.Waits.alignmentTimeoutSeconds),
            Commands.either(
                setRollerVoltage(intakeRoller, voltage),
                Commands.run(() -> {}, intakeRoller),
                readyToFeed))
        .withName("IntakeRollerVoltageAfterWait");
  }
}
