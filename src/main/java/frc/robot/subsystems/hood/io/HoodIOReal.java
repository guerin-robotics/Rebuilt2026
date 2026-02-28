package frc.robot.subsystems.hood.io;

import edu.wpi.first.wpilibj.PWM;
import frc.robot.HardwareConstants;

public class HoodIOReal implements HoodIO {

  private final PWM hoodServo;
  private final PWM hoodLeftServo;

  public HoodIOReal() {
    hoodServo = new PWM(HardwareConstants.CanIds.HOOD_SERVO_CHANNEL);
    hoodLeftServo = new PWM(HardwareConstants.CanIds.HOOD_LEFT_SERVO_CHANNEL);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.servoSpeed = hoodServo.getSpeed();
    inputs.servoPos = hoodServo.getPosition();
  }

  public void setHoodPos(double position) {
    hoodServo.setPosition(position);
    hoodLeftServo.setPosition(position);
  }

  public void stopHood() {
    hoodServo.setSpeed(0);
    hoodLeftServo.setSpeed(0);
  }
}
