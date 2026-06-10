package frc.robot.subsystems.example;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.example.io.ExampleSubsystemIO;
import frc.robot.subsystems.example.io.ExampleSubsystemIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/**
 * Template subsystem following the Guerin Robotics AdvantageKit IO pattern.
 *
 * <p>TEMPLATE INSTRUCTIONS: 1. Rename "ExampleSubsystem" → your subsystem name 2. Update the
 * Logger.processInputs() key to match your subsystem name 3. Update the batteryLogger key to match
 * your subsystem name 4. Add public methods for every control action 5. Do NOT add hardware calls
 * here — all hardware goes in the IO implementation 6. Do NOT reference other subsystems — use
 * RobotState or Supplier callbacks
 *
 * <p>RULE: This class must compile and run identically whether the IO is real or sim. That's the
 * whole point of the IO abstraction.
 */
public class ExampleSubsystem extends SubsystemBase {

  private final ExampleSubsystemIO io;
  private final ExampleSubsystemIOInputsAutoLogged inputs;

  public ExampleSubsystem(ExampleSubsystemIO io) {
    this.io = io;
    this.inputs = new ExampleSubsystemIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ExampleSubsystem", inputs); // TODO: rename key

    // Report current to battery logger for brownout diagnosis
    Robot.batteryLogger.reportCurrentUsage(
        "ExampleSubsystem", // TODO: rename
        false,
        inputs.motorSupplyAmps != null ? inputs.motorSupplyAmps.in(Amps) : 0.0);
  }

  // --- Public control methods (called by commands) ---

  public void setVoltage(Voltage volts) {
    io.setVoltage(volts);
  }

  public void setVelocity(AngularVelocity velocity) {
    io.setVelocity(velocity);
  }

  public void stop() {
    io.setVoltage(Volts.of(0));
  }

  // --- State queries (used by Triggers or commands) ---

  // Example: public boolean isAtVelocity() { return ... }
}
