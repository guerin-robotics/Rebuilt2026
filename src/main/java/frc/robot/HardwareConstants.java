package frc.robot;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.AllianceFlipUtil;

public class HardwareConstants {
  public static final int NUMBER_OF_CAMERAS = 4;

  public static class CanIds {
    // Flywheel
    public static int MAIN_FLYWHEEL_LEADER_ID = 30;
    public static int MAIN_FLYWHEEL_FOLLOWER1_ID = 31;
    public static int MAIN_FLYWHEEL_FOLLOWER2_ID = 32;
    public static int MAIN_FLYWHEEL_FOLLOWER3_ID = 33;
    public static int MAIN_FLYWHEEL_FOLLOWER4_ID = 34;

    // Hood
    public static int HOOD_MOTOR = 35;
    public static int HOOD_ENCODER = 50;

    // Prestage
    public static int PRESTAGE_LEADER_ID = 38;
    public static int PRESTAGE_FOLLOWER_ID = 37;

    // Feeder
    public static int UPPER_FEEDER_MOTOR_ID = 36;
    public static int LOWER_FEEDER_MOTOR_ID = 39;

    // Transport
    public static int TRANSPORT_MOTOR_ID = 40;

    // Intake
    public static int INTAKE_PIVOT_MOTOR_ID = 41;
    public static int INTAKE_ROLLER_LEADER_ID = 42;
    public static int INTAKE_ROLLER_FOLLOWER_ID = 43;
    public static int INTAKE_PIVOT_ENCODER_ID = 44;
  }

  public static class CompConstants {
    // Subsystems that run at a constant voltage: transport, roller
    public static class Voltages {
      public static final Voltage transportVoltage = Volts.of(-7.0);
      public static final Voltage intakeRollerVoltage = Volts.of(12.0);
      public static final Voltage intakeRollerAgitateVoltage = Volts.of(3);
      public static final Voltage prestageIdleVoltage = Volts.of(-1);
      public static final Voltage prestageVoltage = Volts.of(8);
    }

    public static class SpitVoltages {
      public static final Voltage transportSpitVoltage = Volts.of(12);
      public static final Voltage intakeRollerSpitVoltage = Volts.of(10);
    }

    // Subsystems that run at a constant velocity: prestage, feeder
    public static class Velocities {
      public static final AngularVelocity prestageVelocity = RPM.of(2400.0);
      public static final AngularVelocity feederVelocity = RPM.of(-3000.0);
      public static final AngularVelocity prestageIdleVelocity = RPM.of(1080);
      public static final AngularVelocity flywheelIdleVelocity = RPM.of(900);
      public static final AngularVelocity prestageIdleVelocityHigh = RPM.of(1200);
      public static final AngularVelocity flywheelIdleVelocityHigh = RPM.of(60);
      public static final AngularVelocity intakeRollerVelocity = RPM.of(2400);
      public static final AngularVelocity transportVelocity = RPM.of(-1800);
    }

    public static class SpitVelocities {
      public static final AngularVelocity prestageSpitVelocity = RotationsPerSecond.of(50.0);
      public static final AngularVelocity feederSpitVelocity = RotationsPerSecond.of(50.0);
      public static final AngularVelocity flywheelSpitVelocity = RotationsPerSecond.of(17);
    }

    // Subsystems that run on position control: hood, pivot
    public static class Positions {
      public static final Angle hoodDownPos = Degrees.of(0);
      public static final Angle pivotUpPos = Rotations.of(0.3);
      public static final Angle pivotDownPos = Rotations.of(0.0);
      public static final Angle pivotJostleUpPos = Rotations.of(0.25);
      public static final Angle pivotJostleMiddlePos = Rotations.of(0.125);
    }

    public static class Waits {
      public static final double flywheelSpinupSeconds = 0.5;
      public static final double passSpinUpSeconds = 0.75;
      public static final double waitToCompressSeconds = 0.5;
      public static final double waitBetweenCompressSeconds = 0.75;
      public static final double autoWaitToCompressSeconds = 0.75;
      public static final double autoWaitBetweenCompressSeconds = 1.0;
    }

    public static class Thresholds {
      public static final double flywheelSpinupThreshold = 200;
    }
  }

  public static class TestConstants {

    public static class TestVoltages {
      public static final Voltage FlywheelTestVoltage = Volts.of(10.0); // Volts
      public static final Voltage FeederTestVoltage = Volts.of(6.0);
      public static final Voltage LowerFeederTestVoltage = Volts.of(-6.0);
      public static final Voltage PrestageTestVoltage = Volts.of(8.0);
      public static final Voltage TransportTestVoltage = Volts.of(-12.0);
      public static final Voltage intakePivotTestVoltageUp = Volts.of(2.0);
      public static final Voltage intakePivotTestVoltageDown = Volts.of(-8.0);
      public static final Voltage intakeRollerTestVoltage = Volts.of(12.0);
      public static final Voltage intakeRollerAgitateVoltage = Volts.of(3);
    }

    public static class TestVelocities {
      public static final AngularVelocity FlywheelVelocity = RPM.of(600.0);
      public static final AngularVelocity flywheelIdleVelocity = RPM.of(30.0);
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
      // Hood position in degrees (converted from old 0.0-1.0 rotation range × 360)
      public static final Angle hoodPos1Test = Degrees.of(36); // was 0.1 rot
      public static final Angle hoodPos2Test = Degrees.of(90); // was 0.25 rot
      public static final Angle hoodPos3Test = Degrees.of(180); // was 0.5 rot
      public static final Angle hoodPos4Test = Degrees.of(270); // was 0.75 rot
      public static final Angle hoodPos5Test = Degrees.of(360); // was 1.0 rot
    }
  }

  public static class TowerConstants {
    public static final AngularVelocity FlywheelTowerVelocity = RPM.of(1625);
    public static final Angle hoodTowerPos = Degrees.of(2.5); // was 0.55 rot → 198°
  }

  public static class PassConstants {
    public static final AngularVelocity FlywheelPassVelocity = RPM.of(2000);
    public static final Angle hoodPassPos = Degrees.of(10.0);
  }

  public static class TuningConstants {
    public static boolean TUNING_MODE;
    public static boolean isTuning = false;
    public static boolean atComp = true;

    public static final AngularVelocity FlywheelTuningVelocity = RPM.of(1625.0);
    public static final Angle HoodTuningPos = Degrees.of(2.5);
  }

  public static class ControllerConstants {
    public static final int XboxControllerPort = 1;
    public static final int JoystickControllerPort = 2;
    public static final int ButtonPanelPort = 0;
    public static final double DEADBAND = 0.08;
    public static final int SimControllerPort = 5;
  }

  public static class Zones {

    public enum broadZone {
      ALLIANCE_ZONE,
      ALLIANCE_TRENCH,
      NEUTRAL,
      OPPOSING_TRENCH,
      OPPOSING_ZONE
    }

    public enum specificZone {
      ALLIANCE_TOWER,
      ALLIANCE_TRENCH_NEAR,
      ALLIANCE_BUMP_NEAR,
      ALLIANCE_HUB,
      ALLIANCE_BUMP_FAR,
      ALLIANCE_TRENCH_FAR,
      OPPOSING_TRENCH_NEAR,
      OPPOSING_BUMP_NEAR,
      OPPOSING_HUB,
      OPPOSING_BUMP_FAR,
      OPPOSING_TRENCH_FAR,
      OPPOSING_TOWER
    }

    public enum approachingZoneX {
      APPROACHING_ALLIANCE_TOWER,
      APPROACHING_ALLIANCE_TRENCH,
      APPROACHING_OPPOSING_TRENCH,
      APPROACHING_OPPOSING_TOWER
    }

    public enum approachingZoneY {
      APPROACHING_ALLIANCE_TOWER,
      APPROACHING_TRENCH,
      APPROACHING_BUMP,
      APPROACHING_OPPOSING_TOWER,
      APPROACHING_WALL
    }

    public enum approachingZoneComposite {
      APPROACHING_ALLIANCE_TOWER,
      APPROACHING_ALLIANCE_TRENCH,
      APPROACHING_ALLIANCE_BUMP,
      APPROACHING_OPPOSING_BUMP,
      APPROACHING_OPPOSING_TRENCH,
      APPROACHING_OPPOSING_TOWER
    }

    public static final double zoneOffset = AllianceFlipUtil.applyX(0.5);
    public static final double timeInterval = 0.05;

    public static final double approachingXOffset = AllianceFlipUtil.applyX(0.5);
    public static final double approachingYOffset = AllianceFlipUtil.applyY(0.5);
  }

  public static class hubDangerZone {
    public static final double intakeOffset = inchesToMeters(60);
  }
}
