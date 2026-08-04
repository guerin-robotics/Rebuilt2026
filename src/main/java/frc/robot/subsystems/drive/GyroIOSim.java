package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.wpilibj.Timer;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;

/**
 * Gyro IO backed by a MapleSim {@link GyroSimulation}.
 *
 * <p>Replaces the anonymous {@code new GyroIO() {}} previously used in simulation, which reported
 * {@code connected = false} and left {@code Drive} permanently falling back to deriving heading
 * from module deltas. That fallback hides an entire class of bug: with no gyro, a robot that is
 * spun by a collision or slips its wheels still reports a perfect heading, because kinematics
 * cannot see what the wheels did not measure.
 *
 * <p>This implementation reports a connected Pigeon2-like gyro with realistic drift, so heading
 * error behaves the way it does on the field.
 */
public class GyroIOSim implements GyroIO {

  private final GyroSimulation gyroSimulation;

  public GyroIOSim(GyroSimulation gyroSimulation) {
    this.gyroSimulation = gyroSimulation;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = gyroSimulation.getGyroReading();
    inputs.yawVelocityRadPerSec = gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond);

    inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
    inputs.odometryYawTimestamps =
        getSimulationOdometryTimestamps(inputs.odometryYawPositions.length);
  }

  /**
   * Timestamps for each cached sub-tick sample, evenly spaced across the robot period that just
   * ended. Must line up with {@link ModuleIOMapleSim}'s timestamps — {@code Drive.periodic()}
   * indexes gyro and module samples together.
   */
  private static double[] getSimulationOdometryTimestamps(int sampleCount) {
    int subTicks = Math.max(sampleCount, SimulatedArena.getSimulationSubTicksIn1Period());
    double[] timestamps = new double[sampleCount];
    double periodSeconds = 0.02;
    double now = Timer.getFPGATimestamp();
    for (int i = 0; i < sampleCount; i++) {
      timestamps[i] = now - periodSeconds + periodSeconds * (i + 1) / subTicks;
    }
    return timestamps;
  }
}
