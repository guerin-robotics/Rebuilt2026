package frc.robot.subsystems.hood.io;

import edu.wpi.first.wpilibj.PWM;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.hood.HoodConstants;

public class HoodIOReal implements HoodIO {

  private final PWM hoodServo;
  private final PWM hoodLeftServo;

  private final TalonFX hoodMotor;

  public HoodIOReal() {
    hoodServo = new PWM(HardwareConstants.CanIds.HOOD_SERVO_CHANNEL);
    hoodLeftServo = new PWM(HardwareConstants.CanIds.HOOD_LEFT_SERVO_CHANNEL);

    hoodMotor = new TalonFX(HardwareConstants.CanIds.HOOD_MOTOR);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.servoSpeed = hoodServo.getSpeed();
    inputs.servoPos = hoodServo.getPosition();
  }

  @Override
  public void setHoodPos(double position) {
    hoodServo.setPosition(position);
    hoodLeftServo.setPosition(position + HoodConstants.Mechanical.leftServoOffset);
  }

  public void stopHood() {
    hoodServo.setSpeed(0);
    hoodLeftServo.setSpeed(0);
  }
}
