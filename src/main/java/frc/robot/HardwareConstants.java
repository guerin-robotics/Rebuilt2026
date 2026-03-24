package frc.robot;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class HardwareConstants {
  public static final int NUMBER_OF_CAMERAS = 4;

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
    public static int INTAKE_PIVOT_MOTOR_ID = 39;
    public static int INTAKE_ROLLER_LEADER_ID = 40;
    public static int INTAKE_ROLLER_FOLLOWER_ID = 41;
    public static int INTAKE_PIVOT_ENCODER_ID = 42;

    // Hood
    public static int HOOD_SERVO_CHANNEL = 0;
    public static int HOOD_LEFT_SERVO_CHANNEL = 1;
  }

  public static class CompConstants {
    // Subsystems that run at a constant voltage: transport, roller
    public static class Voltages {
      public static final Voltage transportVoltage = Volts.of(-12.0);
      public static final Voltage intakeRollerVoltage = Volts.of(12.0);
      public static final Voltage intakeRollerAgitateVoltage = Volts.of(3);
    }

    public static class SpitVoltages {
      public static final Voltage transportSpitVoltage = Volts.of(12);
      public static final Voltage intakeRollerSpitVoltage = Volts.of(10);
    }

    // Subsystems that run at a constant velocity: prestage, feeder
    public static class Velocities {
      public static final AngularVelocity prestageVelocity = RotationsPerSecond.of(100.0);
      public static final AngularVelocity feederVelocity = RotationsPerSecond.of(180.0);
      public static final AngularVelocity flywheelIdleVelocity = RotationsPerSecond.of(5);
    }

    public static class SpitVelocities {
      public static final AngularVelocity prestageSpitVelocity = RotationsPerSecond.of(50.0);
      public static final AngularVelocity feederSpitVelocity = RotationsPerSecond.of(50.0);
      public static final AngularVelocity flywheelSpitVelocity = RotationsPerSecond.of(17);
    }

    // Subsystems that run on position control: hood, pivot
    public static class Positions {
      public static final double hoodDownPos = 0.1;
      public static final double pivotUpPos = 0.45;
      public static final double pivotDownPos = 0.075;
      public static final double pivotJostleUpPos = 0.25;
    }

    public static final double flywheelSpinupSeconds = 0.3;
  }

  public static class TestConstants {

    public static class TestVoltages {
      public static final Voltage FlywheelTestVoltage = Volts.of(6.0); // Volts
      public static final Voltage FeederTestVoltage = Volts.of(3.0);
      public static final Voltage PrestageTestVoltage = Volts.of(5.0);
      public static final Voltage TransportTestVoltage = Volts.of(-12.0);
      public static final Voltage intakePivotTestVoltageUp = Volts.of(10.0);
      public static final Voltage intakePivotTestVoltageDown = Volts.of(-8.0);
      public static final Voltage intakeRollerTestVoltage = Volts.of(12.0);
      public static final Voltage intakeRollerAgitateVoltage = Volts.of(3);
    }

    public static class TestVelocities {
      public static final AngularVelocity FlywheelVelocity = RPM.of(600); // rotations/s
      public static final AngularVelocity flywheelIdleVelocity = RotationsPerSecond.of(5);
      public static final AngularVelocity feederVelocity = RotationsPerSecond.of(180.0);
      public static final AngularVelocity rollerVelocity = RotationsPerSecond.of(-50.0);
      public static final AngularVelocity rollerAgitateVelocity = RotationsPerSecond.of(-25);
      public static final AngularVelocity prestageVelocity = RotationsPerSecond.of(100.0);
      public static final AngularVelocity transportIntakeVelocity = RotationsPerSecond.of(0.0);
      public static final AngularVelocity transportVelocity = RotationsPerSecond.of(-100.0);
      public static final AngularVelocity pivotUpVelocity = RotationsPerSecond.of(60.0);
      public static final AngularVelocity pivotDownVelocity = RotationsPerSecond.of(-60.0);
    }

    public static class SpitVelocities {
      public static final AngularVelocity FlywheelSpitTestVelocity =
          RotationsPerSecond.of(17); // rotations/s
      public static final AngularVelocity feederSpitTestVelocity = RotationsPerSecond.of(50.0);
      public static final AngularVelocity rollerSpitTestVelocity = RotationsPerSecond.of(50.0);
      public static final AngularVelocity prestageSpitTestVelocity = RotationsPerSecond.of(50.0);
      public static final AngularVelocity transportSpitTestVelocity = RotationsPerSecond.of(-10.0);
    }

    public static class SpitVoltages {
      public static final Voltage rollerSpitTestVolts = Volts.of(10);
      public static final Voltage transportSpitTestVolts = Volts.of(12);
    }

    public static class TestPositions {
      // Pivot pulse seconds
      public static final double pulseSeconds = 0.25;
      // Pivot extension test degrees
      public static final double intakeDegreesUpTest = 0.45;
      public static final double intakeDegreesDownTest = 0.075;
      public static final double intakeJostleTest = 0.25;
      // Hood position (0.0-1.0)
      public static final double hoodPos1Test = 0.1;
      public static final double hoodPos2Test = 0.25;
      public static final double hoodPos3Test = 0.5;
      public static final double hoodPos4Test = 0.75;
      public static final double hoodPos5Test = 1.0;
    }
  }

  public static class TowerConstants {
    public static final AngularVelocity FlywheelTowerVelocity = RPM.of(2300);
    public static final double hoodTowerPos = 0.5;
  }

  public static class PassConstants {
    public static final AngularVelocity FlywheelPassVelocity = RPM.of(2800);
    public static final double hoodPassPos = 0.75;
  }

  public static class TuningConstants {
    public static boolean TUNING_MODE;
    public static boolean isTuning = false;

    public static final AngularVelocity FlywheelTuningVelocity = RPM.of(1700);
    public static final double HoodTuningPos = 0.45;
  }

  public static class ControllerConstants {
    public static final int XboxControllerPort = 1;
    public static final int JoystickControllerPort = 2;
    public static final int ButtonPanelPort = 0;
    public static final double DEADBAND = 0.08;
  }

  public static class Zones {

    public enum Zone {
      ALLIANCE_ZONE,
      APPROACHING_ALLIANCE_TRENCH,
      ALLIANCE_TRENCH_NEAR,
      ALLIANCE_HUB,
      ALLIANCE_TRENCH_FAR,
      NEUTRAL,
      OPPOSING_TRENCH_NEAR,
      OPPOSING_HUB,
      OPPOSING_TRENCH_FAR,
      APPROACHING_OPPOSING_TRENCH,
      OPPOSING_ZONE
    }

    public enum IntakeZone {
      INTAKE_DANGER,
      INTAKE_SAFE
    }

    public static final double zoneOffset = 0.3;
    public static final double timeInterval = 0.25;
  }

  public static class hubDangerZone {
    public static final double intakeOffset = inchesToMeters(25.5);
  }
}
