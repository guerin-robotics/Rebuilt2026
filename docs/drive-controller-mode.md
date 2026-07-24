# Drive Controller Mode — Flightstick / Xbox Swap

How to switch which controller drives the robot between matches, without a redeploy.

Use this when the drive team is rotating drivers at an event and one driver uses the
flightstick while another uses the Xbox controller.

**The two modes:**

| Mode | Drives the robot | Override controller |
|---|---|---|
| `FLIGHTSTICK Drive (normal)` | Flightstick (Thrustmaster) | Xbox |
| `XBOX CONTROLLER Drive` | Xbox | Flightstick |

Both controllers stay plugged in for both modes. Only their roles swap.

---

## 1. Driver station port check (do this once per event)

| Port | Device |
|---|---|
| 1 | Xbox controller |
| 2 | Flightstick |

Confirm this in the Driver Station USB tab before the first match. If a controller
gets unplugged and replugged it can land on a different port, and the symptom is
"my controller does nothing" — not an error message.

---

## 2. Elastic setup (do this once, then save the layout)

Three widgets, all under **SmartDashboard**. They are published as soon as robot code
starts, so you don't need to enable to see them.

1. Open Elastic and connect to the robot (gear icon → team number 10021).
2. Go to the **Teleoperated** tab — the code auto-selects this tab on enable, so this
   is where the drive team will already be looking.
3. **Add Widget → NetworkTables → SmartDashboard**, then drag on:

| Key | Widget type | Shows |
|---|---|---|
| `DRIVE CONTROLLER` | ComboBox Chooser (automatic) | the dropdown you pick from |
| `DRIVE NEXT ENABLE` | Text Display | what will drive at the next enable |
| `DRIVING NOW` | Text Display | what is driving right now |

4. Stack them vertically in that order. Make `DRIVING NOW` the largest — it is the one
   that answers "which controller do I pick up."
5. **Save the layout** (Ctrl+S / File → Save Layout) or you will rebuild this at the event.

---

## 3. Switching drivers between matches

Do this with the robot **disabled**.

1. Pick the incoming driver's controller from the `DRIVE CONTROLLER` dropdown:
   - `FLIGHTSTICK Drive (normal)`
   - `XBOX CONTROLLER Drive`
2. Watch `DRIVE NEXT ENABLE`. Within a moment it should name the controller you picked.
   If it does not change, the dashboard is not connected to the robot — fix that first.
3. Enable teleop. `DRIVING NOW` updates to match.

### The one rule

> **`DRIVING NOW` and `DRIVE NEXT ENABLE` must say the same thing before the match starts.**

If they disagree, the change has not taken effect yet. Disable and re-enable.

They only disagree in the window between changing the dropdown and the next enable —
which is exactly the state worth catching.

### Why the change doesn't apply immediately

The selection is read **once**, at the moment teleop is enabled, and never again during
the match. This is deliberate: checking it continuously would cost loop time on every
button press the robot reads. Changing the dropdown mid-match does nothing until the
next disable → enable.

---

## 4. What happens after a brownout or reboot

The selection is saved to the roboRIO's flash. If the RIO resets mid-match, robot code
restarts and **comes back on the same controller you selected**. You do not need to do
anything.

`DRIVING NOW` will name the restored controller rather than reading
`-- NOT ENABLED YET --`, so you can confirm the recovery took.

This also survives a full power cycle and a code redeploy.

> **The selection is sticky across events.** It stays wherever it was last set — including
> days later at the next event. If you finished the last event on Xbox, the robot boots on
> Xbox. Always confirm `DRIVE NEXT ENABLE` before your first match.

---

## 5. Button map

### Xbox drive mode

| Control | Function |
|---|---|
| Left stick | Translation (drive) |
| Right stick X | Rotation |
| Right trigger | Shoot |
| Left trigger | Intake roller |
| Left bumper | Intake in (pivot up) |
| Right bumper | Intake out (pivot down) |
| Right stick press | Trench align |
| A | Bump align |
| Y | Shoot from tower |

The flightstick becomes the override controller:

| Flightstick button | Function |
|---|---|
| 2 | Disable hub shift logic |
| 3 | Flip alliance winner |
| 4 | Flip alliance winner (same as 3) |
| 6 | Manual compress |
| 7 | Demo shot (demo mode only) |
| 11 | Pass |
| 12 | Cancel auto-X |

### Flightstick drive mode (normal)

Unchanged from what the team has always run — see `Triggers.java` for the full list.
The Xbox is the override controller: A flips alliance winner, Y disables hub shift,
B toggles double compress.

### Not available in Xbox drive mode

These stay on the flightstick and have no Xbox button:

- **Pass** — reachable only via the shoot button when outside the alliance zone. An Xbox
  driver cannot pass while standing inside the alliance zone.
- **Manual compress** — auto-compress on shoot still works.
- **Demo / tuning / hardstop shots** — practice and demo only.

If any of these turn out to matter in a real match, Xbox X, the D-pad, Start, Back, and
the left stick press are all still free.

---

## 6. Changing a button binding

Button bindings are **code**, not a dashboard setting. Changing one is a
`Triggers.java` edit plus a redeploy — unlike swapping which controller drives, which
is the runtime dropdown in section 3. If you only need to hand the robot to a different
driver, use the dropdown; don't touch this.

Every function has one method in `Triggers.java`, in one of three shapes.

**1. Drive-facing functions** (shoot, intake, aligns) use `sourced(...)`:

```java
public Trigger shootFromTowerButton() {
    // Y in Xbox mode
    return sourced(thrustmaster.button(10), controller.y());
    //             ^ flightstick-drive mode   ^ Xbox-drive mode
}
```

The **first** argument is the binding when the flightstick drives; the **second** is
the binding in Xbox drive mode. To move this onto Xbox X, change `controller.y()` to
`controller.x()` (and fix the `// Y in Xbox mode` comment).

**2. Override functions** (`allianceWinFlipper`, `allianceWinDisabler`) also use
`sourced(...)`, but the arguments read the opposite way — these live on the
*non-driving* controller, so in Xbox drive mode they sit on the flightstick:

```java
public Trigger allianceWinDisabler() {
    return sourced(controller.y(), thrustmaster.button(2));
    //             ^ flightstick-drive: Xbox Y   ^ Xbox-drive: flightstick btn 2
}
```

For the Xbox-mode flightstick button, edit the **second** argument.

**3. Single-controller functions** (`passButton`, `intakeCompressButton`,
`autoXOverride`) return one binding with no `sourced(...)`, and that binding is live in
**both** modes. `passButton` and `intakeCompressButton` stay on the flightstick in both
modes — this is why they are the "not available on Xbox" gaps above.

`doubleCompressOverride` (Xbox B) is override-only: it uses `sourced(...)` with a dead
second argument (`new Trigger(() -> false)`), so it toggles double compress **only when
the flightstick drives**. In Xbox drive mode B does nothing — the Xbox driver has no
hidden toggle on the pad, and double compress is not available in that mode.

### Binding names

| Physical control | Code |
|---|---|
| A / B / X / Y | `controller.a()` … `controller.y()` |
| Left / right bumper | `controller.leftBumper()` / `.rightBumper()` |
| Left / right trigger | `controller.leftTrigger(TRIGGER_THRESHOLD)` / `.rightTrigger(TRIGGER_THRESHOLD)` |
| Left / right stick press | `controller.leftStick()` / `.rightStick()` |
| D-pad | `controller.povUp()` / `.povDown()` / `.povLeft()` / `.povRight()` |
| Start / Back | `controller.start()` / `.back()` |
| Flightstick button N | `thrustmaster.button(N)` |

**Free Xbox buttons** — nothing on them in Xbox drive mode: X, the D-pad, Start, Back,
and the left stick press. Prefer these when adding a function.

### Rules

- **Do not put two live functions on the same button in the same mode.** The
  `sourced(...)` design lets a button mean different things across modes safely — Xbox
  A is bump align while driving and alliance-flip while overriding — but two
  drive-mode functions on Xbox A would both fire.
- **Drive axes go through `driveXSupplier()` / `driveYSupplier()` /
  `driveRotSupplier()`**, not `RobotContainer`. A sign flip there is inherited by the
  align commands too (see section 7).
- After the edit: `./gradlew compileJava`, then `./gradlew spotlessApply`, then
  redeploy. **Update the section 5 button map** so the tables don't drift from the code.

---

## 7. First time on the robot — verify stick direction

Before trusting Xbox mode in a match, enable in a safe open space and push the left stick
gently forward.

Xbox left-stick Y is conventionally inverted relative to a flight stick, so **forward may
come out backwards**. If it does, the fix is a sign flip in `driveYSupplier()` in
`Triggers.java` — not in `RobotContainer` — so the align commands inherit it too.

Check rotation direction the same way with the right stick.

---

## 8. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Dropdown change does nothing to `DRIVE NEXT ENABLE` | Dashboard not connected | Reconnect Elastic; check team number |
| `DRIVING NOW` disagrees with `DRIVE NEXT ENABLE` | Selection made but not yet latched | Disable, re-enable |
| Controller does nothing at all | Wrong DS port | Check DS USB tab: Xbox on 1, flightstick on 2 |
| Robot drives backwards in Xbox mode | Stick sign convention | See section 7 |
| Robot boots on the wrong controller | Sticky saved selection | Re-pick from the dropdown |
| Widgets missing in Elastic | Robot code not running | Confirm code is deployed and running |

---

## Where this lives in code

| File | Role |
|---|---|
| `HardwareConstants.ControllerConstants` | `XBOX_DRIVE_MODE` flag, dashboard keys, display names |
| `Triggers.java` | `sourced()` routing, `driveXSupplier`/`driveYSupplier`/`driveRotSupplier` |
| `RobotContainer.java` | Chooser setup, `isXboxDriveSelected()`, `persistDriveControllerSelection()` |
| `Robot.teleopInit()` | The latch — the only place the selection is read |
| `Robot.disabledPeriodic()` | Updates `DRIVE NEXT ENABLE`, saves selection to flash |
