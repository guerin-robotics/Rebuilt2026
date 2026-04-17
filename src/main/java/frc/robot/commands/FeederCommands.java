package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.lowerFeeder.LowerFeeder;
import frc.robot.subsystems.upperFeeder.UpperFeeder;
import java.util.function.BooleanSupplier;

public class FeederCommands {

  public static Command setLowerFeederVoltage(LowerFeeder lowerFeeder, Voltage voltage) {
    return Commands.startEnd(
            () -> lowerFeeder.setLowerFeederVoltage(voltage), // Apply voltage
            () -> lowerFeeder.setLowerFeederVoltage(Volts.of(0)), // Stop on end
            lowerFeeder)
        .withName("FeederVoltage_" + voltage.in(Volts) + "V");
  }

  public static Command setUpperFeederVoltage(UpperFeeder feeder, Voltage voltage) {
    return Commands.startEnd(
            () -> feeder.setUpperFeederVoltage(voltage), // Apply voltage
            () -> feeder.setUpperFeederVoltage(Volts.of(0)), // Stop on end
            feeder)
        .withName("FeederVoltage_" + voltage.in(Volts) + "V");
  }

  public static Command stopLower(LowerFeeder feeder) {
    return Commands.runOnce(() -> feeder.setLowerFeederVelocity(RotationsPerSecond.of(0)), feeder)
        .withName("FeederStop");
  }

  public static Command stopUpper(UpperFeeder feeder) {
    return Commands.runOnce(() -> feeder.setUpperFeederVelocity(RotationsPerSecond.of(0)), feeder)
        .withName("FeederStop");
  }

  public static Command setLowerFeederVelocity(LowerFeeder feeder, AngularVelocity feederVelo) {
    return Commands.runOnce(() -> feeder.setLowerFeederVelocity(feederVelo), feeder)
        .withName("LowerFeederVelocity");
  }

  public static Command setUpperFeederVelocity(UpperFeeder feeder, AngularVelocity feederVelo) {
    return Commands.runOnce(() -> feeder.setUpperFeederVelocity(feederVelo), feeder)
        .withName("UpperFeederVelocity");
  }

  /**
   * Runs the lower feeder at the given velocity, but only after:
   *
   * <ol>
   *   <li>The flywheel has had {@code flywheelSpinupSeconds} to spin up, AND
   *   <li>The robot is aligned to its target (as reported by {@code isAligned}), OR
   *   <li>{@code alignmentTimeoutSeconds} have elapsed since the button was pressed (failsafe).
   * </ol>
   *
   * <p>This prevents wasting balls by feeding before the robot is facing the target.
   *
   * @param feeder The lower feeder subsystem
   * @param feederVelo The velocity to run at once ready
   * @param isAligned Supplier that returns true when the robot is facing its target
   */
  public static Command setLowerVelocityAfterWait(
      LowerFeeder feeder, AngularVelocity feederVelo, BooleanSupplier isAligned) {
    return Commands.sequence(
            // Always wait for the flywheel to spin up first (fast, 0.3 s).
            new WaitCommand(HardwareConstants.CompConstants.Waits.flywheelSpinupSeconds),
            // Then wait until aimed, but give up after the remaining alignment budget so we
            // never stall a shot. Total max wait = alignmentTimeoutSeconds from button press.
            Commands.waitUntil(isAligned)
                .withTimeout(
                    HardwareConstants.CompConstants.Waits.alignmentTimeoutSeconds
                        - HardwareConstants.CompConstants.Waits.flywheelSpinupSeconds),
            setLowerFeederVelocity(feeder, feederVelo))
        .withName("LowerFeederVelocityAfterWait");
  }

  public static Command setLowerVelocityAfterWait(LowerFeeder feeder, AngularVelocity feederVelo) {
    return setLowerVelocityAfterWait(feeder, feederVelo, () -> true).withName("LowerFeederVelocityAfterWaitNoAlign");
  }

  /**
   * Runs the upper feeder at the given velocity, but only after:
   *
   * <ol>
   *   <li>The flywheel has had {@code flywheelSpinupSeconds} to spin up, AND
   *   <li>The robot is aligned to its target (as reported by {@code isAligned}), OR
   *   <li>{@code alignmentTimeoutSeconds} have elapsed since the button was pressed (failsafe).
   * </ol>
   *
   * @param feeder The upper feeder subsystem
   * @param feederVelo The velocity to run at once ready
   * @param isAligned Supplier that returns true when the robot is facing its target
   */
  public static Command setUpperVelocityAfterWait(
      UpperFeeder feeder, AngularVelocity feederVelo, BooleanSupplier isAligned) {
    return Commands.sequence(
            new WaitCommand(HardwareConstants.CompConstants.Waits.flywheelSpinupSeconds),
            Commands.waitUntil(isAligned)
                .withTimeout(
                    HardwareConstants.CompConstants.Waits.alignmentTimeoutSeconds
                        - HardwareConstants.CompConstants.Waits.flywheelSpinupSeconds),
            setUpperFeederVelocity(feeder, feederVelo))
        .withName("UpperFeederVelocityAfterWait");
  }

    public static Command setUpperVelocityAfterWait(
      UpperFeeder feeder, AngularVelocity feederVelo) {
    return setUpperVelocityAfterWait(feeder, feederVelo, () -> true).withName("UpperFeederVelocityAfterWaitNoAlign");
      }
}
