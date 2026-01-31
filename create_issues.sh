#!/bin/bash

# Script to create 24 GitHub issues for the Rebuilt2026 robotics project
# This script creates issues for 6 subsystems (Climber, Shooter, Hopper, Feeder, Intake, LEDs)
# Each subsystem has 4 issues: Define IO Layer, Implement RealIO Layer, Implement Basic Subsystem, Implement Basic Subsystem Commands

# Repository information
REPO="guerin-robotics/Rebuilt2026"

# Color codes for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Array of subsystems
SUBSYSTEMS=("Climber" "Shooter" "Hopper" "Feeder" "Intake" "LEDs")

# Function to create Issue 1: Define IO Layer
create_io_layer_issue() {
    local subsystem=$1
    local title="${subsystem} - Define IO Layer"
    
    # Special case for LEDs subsystem
    if [ "$subsystem" = "LEDs" ]; then
        local body=$(cat <<EOF
# What

Define the IO interface for the ${subsystem} subsystem following the AdvantageKit IO layer pattern. This interface will define all inputs and outputs for the ${subsystem} hardware abstraction.

# Implementation

Create \`${subsystem}IO.java\` in the appropriate package with:

**Skeleton Example:**
\`\`\`java
package frc.robot.subsystems.${subsystem,,};

import org.littletonrobotics.junction.AutoLog;

public interface ${subsystem}IO {
    @AutoLog
    public static class ${subsystem}IOInputs {
        // Define LED status inputs here
        // Example: public int stripLength = 0;
        // Example: public boolean isActive = false;
        // Example: public int currentPattern = 0;
        // Example: public double brightness = 1.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(${subsystem}IOInputs inputs) {}

    /** Set LED color (RGB) */
    public default void setColor(int r, int g, int b) {}

    /** Set LED pattern */
    public default void setPattern(int patternId) {}

    // Add other control methods as needed
    // Example: public default void setBrightness(double brightness) {}
    // Example: public default void clear() {}
}
\`\`\`

**Key Points:**
- Use \`@AutoLog\` annotation for automatic logging
- Define LED status inputs in the inner class (not motor sensors)
- Include methods for colors, patterns, and effects
- Follow team naming conventions
- Include documentation comments

# Definition of Done

- [ ] \`${subsystem}IO.java\` interface created
- [ ] \`${subsystem}IOInputs\` inner class defined with \`@AutoLog\`
- [ ] LED status fields defined (strip length, active state, pattern, brightness)
- [ ] \`updateInputs()\` method signature defined
- [ ] LED control methods defined (setColor, setPattern, etc.)
- [ ] Code follows team conventions
- [ ] All methods have documentation comments

# Other Resources (If Applicable)

- AdvantageKit Documentation: https://github.com/Mechanical-Advantage/AdvantageKit
- Team coding standards
- WPILib LED Documentation: https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html
EOF
)
    else
        local body=$(cat <<EOF
# What

Define the IO interface for the ${subsystem} subsystem following the AdvantageKit IO layer pattern. This interface will define all inputs and outputs for the ${subsystem} hardware abstraction.

# Implementation

Create \`${subsystem}IO.java\` in the appropriate package with:

**Skeleton Example:**
\`\`\`java
package frc.robot.subsystems.${subsystem,,};

import org.littletonrobotics.junction.AutoLog;

public interface ${subsystem}IO {
    @AutoLog
    public static class ${subsystem}IOInputs {
        // Define sensor inputs here
        // Example: public double positionRad = 0.0;
        // Example: public double velocityRadPerSec = 0.0;
        // Example: public double appliedVolts = 0.0;
        // Example: public double currentAmps = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(${subsystem}IOInputs inputs) {}

    /** Run motors at voltage */
    public default void setVoltage(double volts) {}

    // Add other control methods as needed
    // Example: public default void setPosition(double positionRad) {}
    // Example: public default void stop() {}
}
\`\`\`

**Key Points:**
- Use \`@AutoLog\` annotation for automatic logging
- Define all sensor inputs in the inner class
- Keep methods simple (voltage control, no PID)
- Follow team naming conventions
- Include documentation comments

# Definition of Done

- [ ] \`${subsystem}IO.java\` interface created
- [ ] \`${subsystem}IOInputs\` inner class defined with \`@AutoLog\`
- [ ] Basic input fields defined (position, velocity, voltage, current)
- [ ] \`updateInputs()\` method signature defined
- [ ] Basic control methods defined (e.g., \`setVoltage()\`)
- [ ] Code follows team conventions
- [ ] All methods have documentation comments

# Other Resources (If Applicable)

- AdvantageKit Documentation: https://github.com/Mechanical-Advantage/AdvantageKit
- Team coding standards
- Phoenix 6 Documentation: https://v6.docs.ctr-electronics.com/
EOF
)
    fi

    echo -e "${BLUE}Creating issue: ${title}${NC}"
    gh issue create \
        --repo "$REPO" \
        --title "$title" \
        --body "$body" \
        --label "size: S,priority: Medium"
}

# Function to create Issue 2: Implement RealIO Layer
create_real_io_issue() {
    local subsystem=$1
    local title="${subsystem} - Implement RealIO Layer"
    
    # Special case for LEDs subsystem
    if [ "$subsystem" = "LEDs" ]; then
        local body=$(cat <<EOF
# What

Implement the real hardware IO layer for the ${subsystem} subsystem using appropriate LED control hardware (e.g., CTRE CANdle, AddressableLED, etc.).

# Implementation

Create \`${subsystem}IOHardware.java\` that implements \`${subsystem}IO\`:

**Note**: LEDs subsystem uses different hardware than motor-driven subsystems. Choose appropriate hardware:
- CTRE CANdle for CAN-based LED control
- WPILib AddressableLED for PWM-based LED strips
- Other LED controllers as appropriate

**Skeleton Example (using AddressableLED):**
\`\`\`java
package frc.robot.subsystems.${subsystem,,};

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class ${subsystem}IOAddressableLED implements ${subsystem}IO {
    private final AddressableLED ledStrip;
    private final AddressableLEDBuffer ledBuffer;
    private final int ledCount = 60; // TODO: Use actual LED count from constants

    public ${subsystem}IOAddressableLED() {
        // Initialize LED strip with PWM port
        ledStrip = new AddressableLED(0); // TODO: Replace 0 with actual PWM port from Constants.java
        ledBuffer = new AddressableLEDBuffer(ledCount);
        
        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.setData(ledBuffer);
        ledStrip.start();
    }

    @Override
    public void updateInputs(${subsystem}IOInputs inputs) {
        // Read LED state/status
        // Example: inputs.stripLength = ledBuffer.getLength();
        // Example: inputs.isActive = true;
    }

    @Override
    public void setColor(int r, int g, int b) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
        ledStrip.setData(ledBuffer);
    }

    // Implement other control methods for patterns, animations, etc.
}
\`\`\`

**Key Points:**
- Choose appropriate LED hardware for your robot
- For CANdle: Use CTRE CANdle API
- For AddressableLED: Use WPILib AddressableLED API
- Configure LED count and port/CAN ID from constants
- Implement methods for colors, patterns, and animations
- Update inputs with LED status information

# Definition of Done

- [ ] \`${subsystem}IOHardware.java\` created (name based on hardware choice)
- [ ] LED hardware initialized with correct port/CAN ID
- [ ] \`updateInputs()\` implemented with LED status
- [ ] Color/pattern control methods implemented
- [ ] Code tested with actual hardware (if available)
- [ ] All methods have documentation comments

# Other Resources (If Applicable)

- WPILib AddressableLED: https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html
- CTRE CANdle Documentation: https://v6.docs.ctr-electronics.com/
- Team hardware constants
EOF
)
    else
        local body=$(cat <<EOF
# What

Implement the real hardware IO layer for the ${subsystem} subsystem using CTRE TalonFX motors and Phoenix 6 API.

# Implementation

Create \`${subsystem}IOTalonFX.java\` that implements \`${subsystem}IO\`:

**Skeleton Example:**
\`\`\`java
package frc.robot.subsystems.${subsystem,,};

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.BaseStatusSignal;

public class ${subsystem}IOTalonFX implements ${subsystem}IO {
    private final TalonFX motor;
    private final VoltageOut voltageControl = new VoltageOut(0);

    public ${subsystem}IOTalonFX() {
        // Initialize motor with CAN ID
        motor = new TalonFX(0); // TODO: Replace 0 with actual CAN ID from Constants.java

        // Configure motor
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // Add other configuration as needed
        motor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(${subsystem}IOInputs inputs) {
        // Read sensor values from motor
        // Example: inputs.positionRad = motor.getPosition().getValueAsDouble();
        // Example: inputs.velocityRadPerSec = motor.getVelocity().getValueAsDouble();
        // Example: inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
        // Example: inputs.currentAmps = motor.getSupplyCurrent().getValueAsDouble();
        
        // Refresh all signals for optimal performance
        BaseStatusSignal.refreshAll(
            // List signals here
        );
    }

    @Override
    public void setVoltage(double volts) {
        motor.setControl(voltageControl.withOutput(volts));
    }

    // Implement other control methods
}
\`\`\`

**Key Points:**
- Use CTRE Phoenix 6 TalonFX API
- Configure motor in constructor (neutral mode, current limits, etc.)
- Use status signals for efficient sensor reading
- Implement all methods from ${subsystem}IO interface
- Use proper CAN IDs from constants file

# Definition of Done

- [ ] \`${subsystem}IOTalonFX.java\` created
- [ ] TalonFX motor initialized with correct CAN ID
- [ ] Motor configuration applied (neutral mode, limits, etc.)
- [ ] \`updateInputs()\` implemented with all sensor readings
- [ ] \`setVoltage()\` implemented using VoltageOut control
- [ ] Status signals refreshed efficiently
- [ ] Code tested with actual hardware (if available)
- [ ] All methods have documentation comments

# Other Resources (If Applicable)

- Phoenix 6 TalonFX API: https://v6.docs.ctr-electronics.com/
- Phoenix 6 Configuration Guide
- Team hardware constants
EOF
)
    fi

    echo -e "${BLUE}Creating issue: ${title}${NC}"
    gh issue create \
        --repo "$REPO" \
        --title "$title" \
        --body "$body" \
        --label "size: M,priority: Low"
}

# Function to create Issue 3: Implement Basic Subsystem
create_subsystem_issue() {
    local subsystem=$1
    local title="${subsystem} - Implement Basic Subsystem"
    local body=$(cat <<EOF
# What

Implement the basic ${subsystem} subsystem class using the AdvantageKit pattern with IO layer abstraction.

# Implementation

Create \`${subsystem}.java\` subsystem class:

**Skeleton Example:**
\`\`\`java
package frc.robot.subsystems.${subsystem,,};

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ${subsystem} extends SubsystemBase {
    private final ${subsystem}IO io;
    private final ${subsystem}IOInputsAutoLogged inputs = new ${subsystem}IOInputsAutoLogged();

    /** Create a new ${subsystem} subsystem. */
    public ${subsystem}(${subsystem}IO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        // Update and log inputs
        io.updateInputs(inputs);
        Logger.processInputs("${subsystem}", inputs);

        // Add any periodic logic here
    }

    /** Run ${subsystem} at specified voltage */
    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    /** Stop the ${subsystem} */
    public void stop() {
        io.setVoltage(0.0);
    }

    // Add other control methods as needed
    // Example: public void runToPosition(double position) { ... }
}
\`\`\`

**Key Points:**
- Extend \`SubsystemBase\`
- Accept IO interface in constructor for hardware abstraction
- Use AutoLogged inputs class
- Update and log inputs in \`periodic()\`
- Provide simple control methods
- No PID or complex control logic

# Definition of Done

- [ ] \`${subsystem}.java\` created extending SubsystemBase
- [ ] Constructor accepts ${subsystem}IO parameter
- [ ] AutoLogged inputs initialized
- [ ] \`periodic()\` method implemented with logging
- [ ] Basic control methods implemented (setVoltage, stop)
- [ ] Code follows team conventions
- [ ] All methods have documentation comments
- [ ] Subsystem can be instantiated in Robot.java

# Other Resources (If Applicable)

- WPILib SubsystemBase documentation
- AdvantageKit logging examples
- Team subsystem patterns
EOF
)

    echo -e "${BLUE}Creating issue: ${title}${NC}"
    gh issue create \
        --repo "$REPO" \
        --title "$title" \
        --body "$body" \
        --label "size: M,priority: Low"
}

# Function to create Issue 4: Implement Basic Subsystem Commands
create_commands_issue() {
    local subsystem=$1
    local title="${subsystem} - Implement Basic Subsystem Commands"
    local body=$(cat <<EOF
# What

Implement basic commands for the ${subsystem} subsystem to enable operator control and testing.

# Implementation

Create command classes for ${subsystem} operations:

**Skeleton Example (Run${subsystem}Command.java):**
\`\`\`java
package frc.robot.commands.${subsystem,,};

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.${subsystem,,}.${subsystem};

/** Command to run the ${subsystem} at a specified voltage */
public class Run${subsystem}Command extends Command {
    private final ${subsystem} subsystem;
    private final double voltage;

    public Run${subsystem}Command(${subsystem} subsystem, double voltage) {
        this.subsystem = subsystem;
        this.voltage = voltage;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        // Optional: add initialization logic
    }

    @Override
    public void execute() {
        subsystem.setVoltage(voltage);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
}
\`\`\`

**Additional Commands to Implement:**
- \`Stop${subsystem}Command\` - Stop the ${subsystem}
- Add other basic commands as needed for ${subsystem} functionality

**Key Points:**
- Commands should be simple and focused
- Use \`addRequirements()\` to declare subsystem dependency
- Clean up in \`end()\` method
- Commands run until interrupted or finished
- Follow WPILib command patterns

# Definition of Done

- [ ] \`Run${subsystem}Command.java\` created
- [ ] \`Stop${subsystem}Command.java\` created
- [ ] Commands properly require the subsystem
- [ ] \`end()\` method stops motors safely
- [ ] Commands can be bound to operator controls
- [ ] Code follows team conventions
- [ ] All classes have documentation comments
- [ ] Commands tested in teleop/auto

# Other Resources (If Applicable)

- WPILib Command-based programming guide
- Team command patterns
- Operator interface documentation
EOF
)

    echo -e "${BLUE}Creating issue: ${title}${NC}"
    gh issue create \
        --repo "$REPO" \
        --title "$title" \
        --body "$body" \
        --label "size: S,priority: Low"
}

# Main script execution
echo -e "${GREEN}================================${NC}"
echo -e "${GREEN}Creating 24 GitHub Issues${NC}"
echo -e "${GREEN}Repository: ${REPO}${NC}"
echo -e "${GREEN}================================${NC}"
echo ""

# Check if gh CLI is authenticated
if ! gh auth status &> /dev/null; then
    echo "Error: GitHub CLI is not authenticated."
    echo "Please run: gh auth login"
    exit 1
fi

# Counter for created issues
count=0

# Create issues for each subsystem
for subsystem in "${SUBSYSTEMS[@]}"; do
    echo -e "${GREEN}--- Creating issues for ${subsystem} subsystem ---${NC}"
    
    create_io_layer_issue "$subsystem"
    ((count++))
    
    create_real_io_issue "$subsystem"
    ((count++))
    
    create_subsystem_issue "$subsystem"
    ((count++))
    
    create_commands_issue "$subsystem"
    ((count++))
    
    echo ""
done

echo -e "${GREEN}================================${NC}"
echo -e "${GREEN}✅ Successfully created ${count} issues!${NC}"
echo -e "${GREEN}================================${NC}"
echo ""
echo "View all issues at: https://github.com/${REPO}/issues"
