package frc.robot.subsystems.hood.io;

import static edu.wpi.first.units.Units.RPM;

/**
 * Simulated implementation of {@link HoodIO}.
 *
 * <p>Simply tracks the servo position in memory — no physics needed for the hood.
 */
public class HoodIOSim implements HoodIO {

  private double simPosition = 0.0;

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // inputs.servoPos = simPosition;
    // inputs.servoSpeed = 0.0;
    inputs.hoodPosition = simPosition;
    inputs.hoodVelocity = RPM.of(0.0);
  }

  @Override
  public void setHoodPos(double position) {
    simPosition = position;
  }
}
