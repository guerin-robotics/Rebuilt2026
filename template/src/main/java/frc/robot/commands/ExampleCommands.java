package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.example.ExampleSubsystem;
import java.util.function.BooleanSupplier;

/**
 * Command factory for ExampleSubsystem.
 *
 * TEMPLATE INSTRUCTIONS:
 * 1. Rename "ExampleSubsystem" → your subsystem name
 * 2. All methods are static — never instantiate this class
 * 3. Always call .withName() for AdvantageKit command logging
 * 4. Use Commands.startEnd() for "run while held, stop on release" behavior
 * 5. Use Commands.runOnce() for one-shot actions
 * 6. Use Commands.run() for continuous actions that should repeat each loop
 * 7. Use Commands.sequence() to chain actions
 *
 * PATTERN: If a command needs to wait for spinup then alignment before acting:
 *
 *   Commands.sequence(
 *       Commands.waitUntil(isSpunUp).withTimeout(spinupTimeout),
 *       Commands.waitUntil(isAligned).withTimeout(alignTimeout - spinupTimeout),
 *       runAction()
 *   )
 *
 * The alignment timeout is intentional — fire anyway if alignment never comes.
 * This is the "competition failsafe" pattern from FeederCommands.java.
 */
public class ExampleCommands {

  // Basic voltage control — runs while command is active, stops on end
  public static Command runAtVoltage(ExampleSubsystem subsystem, Voltage voltage) {
    return Commands.startEnd(
            () -> subsystem.setVoltage(voltage),
            () -> subsystem.stop(),
            subsystem)
        .withName("Example_Voltage_" + voltage.in(Volts) + "V");
  }

  // One-shot velocity set
  public static Command runAtVelocity(ExampleSubsystem subsystem, AngularVelocity velocity) {
    return Commands.runOnce(() -> subsystem.setVelocity(velocity), subsystem)
        .withName("Example_Velocity");
  }

  // Stop immediately
  public static Command stop(ExampleSubsystem subsystem) {
    return Commands.runOnce(() -> subsystem.stop(), subsystem)
        .withName("Example_Stop");
  }

  // Idle (continuous default command pattern — runs forever until interrupted)
  public static Command idle(ExampleSubsystem subsystem) {
    return Commands.run(() -> subsystem.setVoltage(Volts.of(0)), subsystem)
        .withName("Example_Idle");
  }

  // Gated action — waits for spinup, then alignment, then acts
  // Use this pattern when the mechanism must be ready before an action is safe
  public static Command runAfterReady(
      ExampleSubsystem subsystem, AngularVelocity velocity, BooleanSupplier isAligned) {
    return Commands.sequence(
            Commands.waitUntil(() -> subsystem.isAtVelocity()) // TODO: add isAtVelocity() to subsystem
                .withTimeout(HardwareConstants.CompConstants.Waits.flywheelSpinupSeconds),
            Commands.waitUntil(isAligned)
                .withTimeout(
                    HardwareConstants.CompConstants.Waits.alignmentTimeoutSeconds
                        - HardwareConstants.CompConstants.Waits.flywheelSpinupSeconds),
            runAtVelocity(subsystem, velocity))
        .withName("Example_AfterReady");
  }
}
