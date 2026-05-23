# Prompt: Create or Modify an Autonomous Routine

Use this prompt when adding a new auto path or modifying an existing one.

---

## Adding a New Auto (PathPlanner path exists)

```
Add a new autonomous routine called "[AUTO NAME]".

The PathPlanner path file "[filename].auto" already exists in deploy/pathplanner/autos/.

Named commands used in this auto:
- "[CommandName]" — [what it should do, e.g.: "Deploy intake and run rollers"]
- [repeat for each named command]

Register any new named commands in RobotContainer before buildAutoChooser().
Do not change any existing named command registrations.
Do not modify any existing auto paths.
```

---

## Adding a New Named Command

```
Register a new named command "[COMMAND NAME]" for use in PathPlanner autos.

What it should do: [description]

Subsystems involved: [list]

The command should be: [describe — e.g., "parallel: deploy intake + run rollers, ends when intake is up"]

Register it in RobotContainer alongside the existing named commands.
Run ./gradlew compileJava after.
```

---

## Modifying Auto Behavior (not the path, just the command logic)

```
Modify the behavior of the "[COMMAND NAME]" named command used in auto.

Current behavior: [describe what it does now]
Desired behavior: [describe what it should do]
Reason: [why the change is needed]

Only change the command factory method. Do not change the path file.
Do not change any other named commands.
```

---

## Safety Rules for Auto Changes

**Never modify `.auto` path files in code.** PathPlanner paths are edited in the 
PathPlanner GUI, not by hand. Editing the JSON directly corrupts waypoints.

**Test auto start pose check.** After any auto change, verify that 
`robotContainer.checkStartPose()` validates correctly for the new path's start pose.

**Named command registration order matters.** Commands must be registered 
before `AutoBuilder.buildAutoChooser()`. If they're registered after, PathPlanner 
silently ignores them and the path runs without the command.

**Event triggers need no subsystem requirements.** See `.claude/rules/03-commands.md`.

---

## Checking Auto Preview

In `disabledPeriodic()`, the robot calls `robotContainer.updateAutoPreview()` to 
draw the selected auto path on the Field2d widget. After adding a new auto:
1. Select it on the dashboard
2. Verify the path appears correctly on the field visualization
3. Verify the starting pose matches where you intend to place the robot
