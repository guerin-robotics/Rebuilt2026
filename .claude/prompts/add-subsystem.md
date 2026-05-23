# Prompt: Add a New Subsystem

Use this prompt when scaffolding a new mechanism subsystem from scratch.
Copy and fill in the bracketed values before sending.

---

```
Add a new subsystem for [MECHANISM NAME].

Hardware:
- Motor: [TalonFX / other] CAN ID [XX] on ["rio" / "Canivore"] bus
- [Additional motor if follower: TalonFX CAN ID XX, opposes leader: yes/no]
- [Encoder if position-controlled: CANcoder CAN ID XX on [bus]]
- Control mode: [VoltageOut / VelocityTorqueCurrentFOC / MotionMagicTorqueCurrentFOC]

What it needs to do:
- [Action 1, e.g.: run at a set voltage]
- [Action 2, e.g.: run at a target velocity]
- [Action 3 if applicable]

Logged signals needed:
- Voltage, supply current, stator current, velocity, temperature (standard set)
- [Any additional: position, closed-loop reference/error]

State queries needed (for Triggers or commands):
- [e.g.: isAtVelocity() — true when within 200 RPM of target]

Follow the AdvantageKit IO pattern exactly as shown in template/src/.
Create:
1. subsystems/[name]/io/[Name]IO.java
2. subsystems/[name]/io/[Name]IOReal.java  
3. subsystems/[name]/io/[Name]IOSim.java
4. subsystems/[name]/[Name].java
5. commands/[Name]Commands.java

Wire the real and sim implementations in RobotContainer under the existing 
real/sim switch. Add the CAN ID constant to HardwareConstants.CanIds.
Run ./gradlew compileJava before reporting complete.
```

---

## What Claude Will Do

1. Create the five files using the exact IO pattern from `template/`
2. Add CAN ID to `HardwareConstants.CanIds`
3. Add real/sim wiring to `RobotContainer`
4. Provide a compile check

## What Claude Will Not Do Without Asking

- Change existing subsystem code
- Change button bindings
- Add this subsystem to auto sequences
- Change CAN IDs of existing devices
