package frc.robot.subsystems.hood.io;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.hood.HoodConstants;

public class HoodIOReal implements HoodIO {

  private final Servo hoodServo;

  public HoodIOReal() {
    hoodServo = new Servo(HardwareConstants.CanIds.HOOD_SERVO_CHANNEL);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.hoodAngle = hoodServo.getAngle();
    inputs.hoodPos = hoodServo.get();
  }

  public void setHoodPos(double angle) {
    hoodServo.setAngle(angle);
  }
}
