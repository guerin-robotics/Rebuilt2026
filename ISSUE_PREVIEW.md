# Issue Preview - All 24 Issues

This document shows a preview of all 24 issues that will be created.

---

## CLIMBER SUBSYSTEM

### Issue 1: Climber - Define IO Layer
**Labels**: `size: S`, `priority: Medium`

**What**: Define the IO interface for the Climber subsystem following the AdvantageKit IO layer pattern.

**Key Tasks**:
- Create `ClimberIO.java` interface
- Define `ClimberIOInputs` with @AutoLog
- Add sensor input fields (position, velocity, voltage, current)
- Define control methods (setVoltage, etc.)

**DoD**: Interface created with all inputs and methods documented

---

### Issue 2: Climber - Implement RealIO Layer
**Labels**: `size: M`, `priority: Low`

**What**: Implement the real hardware IO layer using CTRE TalonFX motors and Phoenix 6.

**Key Tasks**:
- Create `ClimberIOTalonFX.java` implementing `ClimberIO`
- Initialize and configure TalonFX motor
- Implement updateInputs() with sensor readings
- Implement setVoltage() using VoltageOut control

**DoD**: Real hardware implementation complete and tested

---

### Issue 3: Climber - Implement Basic Subsystem
**Labels**: `size: M`, `priority: Low`

**What**: Implement the basic Climber subsystem class using AdvantageKit pattern.

**Key Tasks**:
- Create `Climber.java` extending SubsystemBase
- Accept IO interface in constructor
- Implement periodic() with logging
- Add basic control methods (setVoltage, stop)

**DoD**: Subsystem instantiable with proper logging

---

### Issue 4: Climber - Implement Basic Subsystem Commands
**Labels**: `size: S`, `priority: Low`

**What**: Implement basic commands for the Climber subsystem.

**Key Tasks**:
- Create `RunClimberCommand.java`
- Create `StopClimberCommand.java`
- Ensure proper requirements and cleanup

**DoD**: Commands can be bound to operator controls

---

## SHOOTER SUBSYSTEM

### Issue 5: Shooter - Define IO Layer
**Labels**: `size: S`, `priority: Medium`

**What**: Define the IO interface for the Shooter subsystem following the AdvantageKit IO layer pattern.

**Key Tasks**:
- Create `ShooterIO.java` interface
- Define `ShooterIOInputs` with @AutoLog
- Add sensor input fields (position, velocity, voltage, current)
- Define control methods (setVoltage, etc.)

**DoD**: Interface created with all inputs and methods documented

---

### Issue 6: Shooter - Implement RealIO Layer
**Labels**: `size: M`, `priority: Low`

**What**: Implement the real hardware IO layer using CTRE TalonFX motors and Phoenix 6.

**Key Tasks**:
- Create `ShooterIOTalonFX.java` implementing `ShooterIO`
- Initialize and configure TalonFX motor
- Implement updateInputs() with sensor readings
- Implement setVoltage() using VoltageOut control

**DoD**: Real hardware implementation complete and tested

---

### Issue 7: Shooter - Implement Basic Subsystem
**Labels**: `size: M`, `priority: Low`

**What**: Implement the basic Shooter subsystem class using AdvantageKit pattern.

**Key Tasks**:
- Create `Shooter.java` extending SubsystemBase
- Accept IO interface in constructor
- Implement periodic() with logging
- Add basic control methods (setVoltage, stop)

**DoD**: Subsystem instantiable with proper logging

---

### Issue 8: Shooter - Implement Basic Subsystem Commands
**Labels**: `size: S`, `priority: Low`

**What**: Implement basic commands for the Shooter subsystem.

**Key Tasks**:
- Create `RunShooterCommand.java`
- Create `StopShooterCommand.java`
- Ensure proper requirements and cleanup

**DoD**: Commands can be bound to operator controls

---

## HOPPER SUBSYSTEM

### Issue 9: Hopper - Define IO Layer
**Labels**: `size: S`, `priority: Medium`

**What**: Define the IO interface for the Hopper subsystem following the AdvantageKit IO layer pattern.

**Key Tasks**:
- Create `HopperIO.java` interface
- Define `HopperIOInputs` with @AutoLog
- Add sensor input fields (position, velocity, voltage, current)
- Define control methods (setVoltage, etc.)

**DoD**: Interface created with all inputs and methods documented

---

### Issue 10: Hopper - Implement RealIO Layer
**Labels**: `size: M`, `priority: Low`

**What**: Implement the real hardware IO layer using CTRE TalonFX motors and Phoenix 6.

**Key Tasks**:
- Create `HopperIOTalonFX.java` implementing `HopperIO`
- Initialize and configure TalonFX motor
- Implement updateInputs() with sensor readings
- Implement setVoltage() using VoltageOut control

**DoD**: Real hardware implementation complete and tested

---

### Issue 11: Hopper - Implement Basic Subsystem
**Labels**: `size: M`, `priority: Low`

**What**: Implement the basic Hopper subsystem class using AdvantageKit pattern.

**Key Tasks**:
- Create `Hopper.java` extending SubsystemBase
- Accept IO interface in constructor
- Implement periodic() with logging
- Add basic control methods (setVoltage, stop)

**DoD**: Subsystem instantiable with proper logging

---

### Issue 12: Hopper - Implement Basic Subsystem Commands
**Labels**: `size: S`, `priority: Low`

**What**: Implement basic commands for the Hopper subsystem.

**Key Tasks**:
- Create `RunHopperCommand.java`
- Create `StopHopperCommand.java`
- Ensure proper requirements and cleanup

**DoD**: Commands can be bound to operator controls

---

## FEEDER SUBSYSTEM

### Issue 13: Feeder - Define IO Layer
**Labels**: `size: S`, `priority: Medium`

**What**: Define the IO interface for the Feeder subsystem following the AdvantageKit IO layer pattern.

**Key Tasks**:
- Create `FeederIO.java` interface
- Define `FeederIOInputs` with @AutoLog
- Add sensor input fields (position, velocity, voltage, current)
- Define control methods (setVoltage, etc.)

**DoD**: Interface created with all inputs and methods documented

---

### Issue 14: Feeder - Implement RealIO Layer
**Labels**: `size: M`, `priority: Low`

**What**: Implement the real hardware IO layer using CTRE TalonFX motors and Phoenix 6.

**Key Tasks**:
- Create `FeederIOTalonFX.java` implementing `FeederIO`
- Initialize and configure TalonFX motor
- Implement updateInputs() with sensor readings
- Implement setVoltage() using VoltageOut control

**DoD**: Real hardware implementation complete and tested

---

### Issue 15: Feeder - Implement Basic Subsystem
**Labels**: `size: M`, `priority: Low`

**What**: Implement the basic Feeder subsystem class using AdvantageKit pattern.

**Key Tasks**:
- Create `Feeder.java` extending SubsystemBase
- Accept IO interface in constructor
- Implement periodic() with logging
- Add basic control methods (setVoltage, stop)

**DoD**: Subsystem instantiable with proper logging

---

### Issue 16: Feeder - Implement Basic Subsystem Commands
**Labels**: `size: S`, `priority: Low`

**What**: Implement basic commands for the Feeder subsystem.

**Key Tasks**:
- Create `RunFeederCommand.java`
- Create `StopFeederCommand.java`
- Ensure proper requirements and cleanup

**DoD**: Commands can be bound to operator controls

---

## INTAKE SUBSYSTEM

### Issue 17: Intake - Define IO Layer
**Labels**: `size: S`, `priority: Medium`

**What**: Define the IO interface for the Intake subsystem following the AdvantageKit IO layer pattern.

**Key Tasks**:
- Create `IntakeIO.java` interface
- Define `IntakeIOInputs` with @AutoLog
- Add sensor input fields (position, velocity, voltage, current)
- Define control methods (setVoltage, etc.)

**DoD**: Interface created with all inputs and methods documented

---

### Issue 18: Intake - Implement RealIO Layer
**Labels**: `size: M`, `priority: Low`

**What**: Implement the real hardware IO layer using CTRE TalonFX motors and Phoenix 6.

**Key Tasks**:
- Create `IntakeIOTalonFX.java` implementing `IntakeIO`
- Initialize and configure TalonFX motor
- Implement updateInputs() with sensor readings
- Implement setVoltage() using VoltageOut control

**DoD**: Real hardware implementation complete and tested

---

### Issue 19: Intake - Implement Basic Subsystem
**Labels**: `size: M`, `priority: Low`

**What**: Implement the basic Intake subsystem class using AdvantageKit pattern.

**Key Tasks**:
- Create `Intake.java` extending SubsystemBase
- Accept IO interface in constructor
- Implement periodic() with logging
- Add basic control methods (setVoltage, stop)

**DoD**: Subsystem instantiable with proper logging

---

### Issue 20: Intake - Implement Basic Subsystem Commands
**Labels**: `size: S`, `priority: Low`

**What**: Implement basic commands for the Intake subsystem.

**Key Tasks**:
- Create `RunIntakeCommand.java`
- Create `StopIntakeCommand.java`
- Ensure proper requirements and cleanup

**DoD**: Commands can be bound to operator controls

---

## LEDS SUBSYSTEM

### Issue 21: LEDs - Define IO Layer
**Labels**: `size: S`, `priority: Medium`

**What**: Define the IO interface for the LEDs subsystem following the AdvantageKit IO layer pattern.

**Key Tasks**:
- Create `LEDsIO.java` interface
- Define `LEDsIOInputs` with @AutoLog
- Add LED status input fields (strip length, active state, pattern, brightness)
- Define control methods (setColor, setPattern, setBrightness, etc.)

**DoD**: Interface created with all inputs and methods documented

---

### Issue 22: LEDs - Implement RealIO Layer
**Labels**: `size: M`, `priority: Low`

**What**: Implement the real hardware IO layer using appropriate LED control hardware (e.g., CTRE CANdle, AddressableLED).

**Key Tasks**:
- Create `LEDsIOHardware.java` implementing `LEDsIO` (name based on hardware choice)
- Initialize and configure LED hardware (CANdle, AddressableLED, etc.)
- Implement updateInputs() with LED status
- Implement color/pattern control methods

**DoD**: Real hardware implementation complete and tested

---

### Issue 23: LEDs - Implement Basic Subsystem
**Labels**: `size: M`, `priority: Low`

**What**: Implement the basic LEDs subsystem class using AdvantageKit pattern.

**Key Tasks**:
- Create `LEDs.java` extending SubsystemBase
- Accept IO interface in constructor
- Implement periodic() with logging
- Add basic control methods (setColor, setPattern, setBrightness)

**DoD**: Subsystem instantiable with proper logging

---

### Issue 24: LEDs - Implement Basic Subsystem Commands
**Labels**: `size: S`, `priority: Low`

**What**: Implement basic commands for the LEDs subsystem.

**Key Tasks**:
- Create `RunLEDsCommand.java`
- Create `StopLEDsCommand.java`
- Ensure proper requirements and cleanup

**DoD**: Commands can be bound to operator controls

---

## Summary

**Total Issues**: 24
**Subsystems**: 6 (Climber, Shooter, Hopper, Feeder, Intake, LEDs)
**Phases per Subsystem**: 4

**Label Distribution**:
- Size: S - 12 issues (IO Layer definition + Commands)
- Size: M - 12 issues (RealIO + Subsystem implementation)
- Priority: Medium - 6 issues (all IO Layer definitions)
- Priority: Low - 18 issues (all implementations)

**File Structure Created**:
```
frc.robot.subsystems.{subsystem}/
├── {Subsystem}IO.java
├── {Subsystem}IOTalonFX.java (or LEDsIOAddressableLED.java for LEDs)
└── {Subsystem}.java

frc.robot.commands.{subsystem}/
├── Run{Subsystem}Command.java
└── Stop{Subsystem}Command.java
```

**Note**: The LEDs subsystem uses AddressableLED or CANdle hardware instead of TalonFX motors.
