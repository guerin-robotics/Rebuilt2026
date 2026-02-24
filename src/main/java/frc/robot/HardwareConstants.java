package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class HardwareConstants {
  public static final int NUMBER_OF_CAMERAS = 2;

  public static class CanIds {
    // Flywheel
    public static int MAIN_FLYWHEEL_LEADER_ID = 31;
    public static int MAIN_FLYWHEEL_FOLLOWER1_ID = 32;
    public static int MAIN_FLYWHEEL_FOLLOWER2_ID = 33;
    public static int MAIN_FLYWHEEL_FOLLOWER3_ID = 34;

    // Prestage
    public static int PRESTAGE_LEADER_ID = 35;
    public static int PRESTAGE_FOLLOWER_ID = 36;

    // Feeder
    public static int FEEDER_MOTOR_ID = 37;

    // Transport
    public static int TRANSPORT_MOTOR_ID = 38;

    // Intake
    public static int INTAKE_SLIDER_MOTOR_ID = 39;
    public static int INTAKE_ROLLER_LEADER_ID = 40;
    public static int INTAKE_ROLLER_FOLLOWER_ID = 41;

    // Climber
    // public static int CLIMBER_MOTOR_ID = 42;
  }

  public static class TestVoltages {
    public static final Voltage FlywheelTestVoltage = Volts.of(6.0); // Volts
    public static final Voltage FeederTestVoltage = Volts.of(3.0);
    public static final Voltage PrestageTestVoltage = Volts.of(5.0);
    public static final Voltage TransportTestVoltage = Volts.of(-3.0);
    public static final Voltage intakeSliderTestVoltage = Volts.of(10.0);
    public static final Voltage intakeSliderTestVoltageIn = Volts.of(-10.0);
    public static final Voltage intakeRollerTestVoltage = Volts.of(10.0);
  }

  public static class TestVelocities {
    public static final AngularVelocity FlywheelVelocity =
        RotationsPerSecond.of(16.75); // rotations/s
    public static final AngularVelocity feederVelocity = RotationsPerSecond.of(40.0);
    public static final AngularVelocity rollerVelocity = RotationsPerSecond.of(-10.0);
    public static final AngularVelocity prestageVelocity = RotationsPerSecond.of(40.0);
    public static final AngularVelocity transportVelocity = RotationsPerSecond.of(-10.0);
    public static final AngularVelocity sliderVelocity = RotationsPerSecond.of(20.0);
    public static final AngularVelocity sliderInVelocity = RotationsPerSecond.of(-20.0);
  }

  public static class PulseConstants {
    // Slider pulse seconds
    public static final double pulseInches = 1.0;
    public static final double pulseSeconds = 0.5;
    // Slider extension test inches
    public static final double intakeInchTest = 3.0;
  }

  public static class ControllerConstants {
    public static final int XboxControllerPort = 1;
    public static final int JoystickControllerPort = 2;
    public static final int ButtonPanelPort = 0;
    public static final double DEADBAND = 0.08;
  }
}
