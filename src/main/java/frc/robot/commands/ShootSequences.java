package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.prestage.Prestage;
import frc.robot.subsystems.hood.Hood;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.transport.Transport;
import frc.robot.subsystems.intakeRoller.intakeRoller;
import frc.robot.subsystems.intakeSlider.intakeSlider;

public class ShootSequences {

    public static Command shootForTower(Flywheel flywheel, Prestage prestage, Hood hood, Feeder feeder,
        Transport transport, intakeRoller intakeRoller) {
            return Commands.parallel(
                Commands.parallel(FlywheelCommands.setFlywheelVelocity(
                    flywheel, HardwareConstants.TowerConstants.FlywheelTowerVelocity),
                    PrestageCommands.setPrestageVelocity(
                        prestage, HardwareConstants.TestVelocities.prestageVelocity),
                    HoodCommands.setHoodPos(hood, HardwareConstants.TowerConstants.hoodTowerPos)
                ),
                Commands.sequence(
                    new WaitCommand(0.5),
                    FeederCommands.setFeederVelocity(feeder, HardwareConstants.TestVelocities.feederVelocity),
                    TransportCommands.setTransportVelocity(
                        transport, HardwareConstants.TestVelocities.transportVelocity),
                    intakeRollerCommands.setRollerVoltage(
                        intakeRoller, HardwareConstants.TestVoltages.intakeRollerAgitateVoltage)
                )
            ).finallyDo(() -> {
                flywheel.setFlywheelVelocity(RotationsPerSecond.of(0));
                prestage.setPrestageVelocity(RotationsPerSecond.of(0));
                hood.stopHood();
                feeder.setFeederVelocity(RotationsPerSecond.of(0));
                transport.setTransportVelocity(RotationsPerSecond.of(0));
                intakeRoller.setRollerVoltage(Volts.of(0));
            });
        }

    public static Command shootForTowerNoDelay(Flywheel flywheel, Prestage prestage, Hood hood,
            Feeder feeder, Transport transport, intakeRoller intakeRoller) {
        return Commands.parallel(
            FlywheelCommands.setFlywheelVelocity(flywheel, HardwareConstants.TowerConstants.FlywheelTowerVelocity),
            PrestageCommands.setPrestageVelocity(prestage, HardwareConstants.TestVelocities.prestageVelocity),
            HoodCommands.setHoodPos(hood, HardwareConstants.TowerConstants.hoodTowerPos),
            FeederCommands.setFeederVelocity(feeder, HardwareConstants.TestVelocities.feederVelocity),
            TransportCommands.setTransportVelocity(transport, HardwareConstants.TestVelocities.transportVelocity),
            intakeRollerCommands.setRollerVoltage(intakeRoller,
                HardwareConstants.TestVoltages.intakeRollerAgitateVoltage)
        );
    }

    public static Command shootByDistance(Flywheel flywheel, Prestage prestage, Hood hood, Feeder feeder,
        Transport transport, intakeRoller intakeRoller) {
        return Commands.parallel(
                Commands.parallel(FlywheelCommands.setVelocityForHub(
                    flywheel),
                    PrestageCommands.setPrestageVelocity(
                        prestage, HardwareConstants.TestVelocities.prestageVelocity),
                    HoodCommands.setHoodPosForHub(hood)
                ),
                Commands.sequence(
                    new WaitCommand(0.5),
                    FeederCommands.setFeederVelocity(feeder, HardwareConstants.TestVelocities.feederVelocity),
                    TransportCommands.setTransportVelocity(
                        transport, HardwareConstants.TestVelocities.transportVelocity),
                    intakeRollerCommands.setRollerVoltage(
                        intakeRoller, HardwareConstants.TestVoltages.intakeRollerAgitateVoltage)
                )
            ).finallyDo(() -> {
                flywheel.setFlywheelVelocity(RotationsPerSecond.of(0));
                prestage.setPrestageVelocity(RotationsPerSecond.of(0));
                hood.stopHood();
                feeder.setFeederVelocity(RotationsPerSecond.of(0));
                transport.setTransportVelocity(RotationsPerSecond.of(0));
                intakeRoller.setRollerVoltage(Volts.of(0));
            }); 
        }

    public static Command FirstSet(Flywheel flywheel, Prestage prestage, Hood hood,
        AngularVelocity flywheelVelo, double hoodPos) {
        return Commands.parallel(
            FlywheelCommands.setFlywheelVelocity(flywheel, flywheelVelo),
            PrestageCommands.setPrestageVelocity(prestage, HardwareConstants.TestVelocities.prestageVelocity),
            HoodCommands.setHoodPos(hood, hoodPos)
        ).finallyDo(() -> {
                flywheel.setFlywheelVelocity(RotationsPerSecond.of(0));
                prestage.setPrestageVelocity(RotationsPerSecond.of(0));
                hood.stopHood();
            }); 
    }

    public static Command SecondSet(Feeder feeder, Transport transport) {
        return Commands.parallel(
            FeederCommands.setFeederVelocity(feeder, HardwareConstants.TestVelocities.feederVelocity),
            TransportCommands.setTransportVelocity(transport, HardwareConstants.TestVelocities.transportVelocity)
        ).finallyDo(() -> {
                feeder.setFeederVelocity(RotationsPerSecond.of(0));
                transport.setTransportVelocity(RotationsPerSecond.of(0));
            }); 
    }
}
