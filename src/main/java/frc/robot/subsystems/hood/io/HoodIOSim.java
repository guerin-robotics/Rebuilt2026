package frc.robot.subsystems.hood.io;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;

/**
 * Simulated implementation of {@link HoodIO}.
 *
 * <p>Simply tracks the servo position in memory — no physics needed for the hood.
 */
public class HoodIOSim implements HoodIO {

  private Angle simPosition = Degrees.of(0);

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // inputs.servoPos = simPosition;
    // inputs.servoSpeed = 0.0;
    inputs.hoodPosition = simPosition;
    inputs.hoodVelocity = RPM.of(0.0);
  }

  @Override
  public void setHoodPos(Angle position) {
    simPosition = position;
  }
}
