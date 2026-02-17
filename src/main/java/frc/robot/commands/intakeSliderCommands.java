package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeSlider.intakeSlider;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj2.command.Commands;

public class intakeSliderCommands {
    
    public static Command runIntakeForward(intakeSlider intakeSlider, Voltage voltage) {
        return Commands.startEnd(() -> intakeSlider.setIntakeSliderVoltage(voltage),
            () -> intakeSlider.setIntakeSliderVoltage(Volts.of(0)), intakeSlider);
    }

    public static Command stopIntakeSlider(intakeSlider intakeSlider) {
        return Commands.runOnce(() -> intakeSlider.setIntakeSliderVoltage(Volts.of(0)), intakeSlider);
    }

}
