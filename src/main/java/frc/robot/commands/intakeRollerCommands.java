package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.intakeRoller.intakeRoller;
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

  public static Command stopIntakeRoller(intakeRoller intakeRoller) {
    return Commands.runOnce(
            () -> intakeRoller.setRollerVelocity(RotationsPerSecond.of(0)), intakeRoller)
        .withName("IntakeRollerStop");
  }

  /**
   * Runs the intake roller at the given agitate voltage, but only after the robot is aligned (or
   * timeout).
   *
   * <p>See {@link FeederCommands#setLowerVelocityAfterWait} for full details on the wait logic.
   *
   * @param intakeRoller The intake roller subsystem
   * @param voltage The agitate voltage to apply once ready
   * @param isAligned Supplier that returns true when the robot is facing its target
   */
  public static Command setVoltageAfterWait(
      intakeRoller intakeRoller, Voltage voltage, BooleanSupplier isAligned) {
    return Commands.sequence(
            new WaitCommand(HardwareConstants.CompConstants.Waits.flywheelSpinupSeconds),
            Commands.waitUntil(isAligned)
                .withTimeout(
                    HardwareConstants.CompConstants.Waits.alignmentTimeoutSeconds
                        - HardwareConstants.CompConstants.Waits.flywheelSpinupSeconds),
            setRollerVoltage(intakeRoller, voltage))
        .withName("IntakeRollerVoltageAfterWait");
  }
}
