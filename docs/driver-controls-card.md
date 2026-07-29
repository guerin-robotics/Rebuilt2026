# Driver Controls — Quick Refresh

Two ways to drive the robot. Someone picks the mode on the dashboard before you enable;
find your mode below and ignore the other one.

```
PLUG-IN CHECK        Port 1  Xbox controller
                     Port 2  Flightstick

If your controller does nothing at all, this is almost always why.
Check the Driver Station USB tab before anything else.

PICKING THE MODE     Dashboard -> "Drive controller" -> Thrustmaster  or  Xbox

It only takes effect at the next ENABLE. Change it mid-match and nothing
happens. After a code redeploy it goes back to Thrustmaster on its own.
```

---

## MODE 1 — FLIGHTSTICK DRIVES  (the usual)

You hold the flightstick. Someone else holds the Xbox as a backup controller.

```
THRUSTMASTER T.16000M
────────────────────────────────────────────────────────────────────────────────

                        STICK HEAD  (looking down)

                          ╭─────────────╮
                         ╱     ╭───╮     ╲
                        │      │ ✛ │      │      the HAT does nothing
         INTAKE IN ─────┤ (3)  ╰───╯  (4) ├───── INTAKE OUT
           pivot up     │                 │        pivot down
                         ╲     (2)       ╱
                          ╰──────┬───────╯        (2) = TRENCH ALIGN
                                 │
                            ┌────┴────┐
                            │   (1)   │
                            └────┬────┘
                                 └───── SHOOT ★   trigger, front of grip

            push / pull / left / right = DRIVE        twist = TURN


                              BASE  (looking down)

        LEFT SIDE                                       RIGHT SIDE
   ┌────────┬────────┬────────┐                  ┌────────┬────────┬────────┐
   │   5    │   6    │   7    │                  │   11   │   12   │   13   │
   │  STOP  │COMPRESS│ demo   │                  │  PASS  │ CANCEL │        │
   │ ROLLER │        │ shot   │                  │        │ AUTO-X │        │
   ├────────┼────────┼────────┤       ⊙ stick    ├────────┼────────┼────────┤
   │   8    │   9    │   10   │                  │   14   │   15   │   16   │
   │  BUMP  │ TURBO  │ TOWER  │                  │        │        │        │
   │ ALIGN  │  ⚡    │  SHOT  │                  │        │        │        │
   └────────┴────────┴────────┘                  └────────┴────────┴────────┘

   Blank buttons do nothing. Button 7 only works in practice/demo mode.
```

| Control | What it does |
|---|---|
| Push / pull / left / right | Drive |
| Twist | Turn |
| **Trigger (1)** | **Shoot** — aims, spins up and fires. See *What the trigger does* below |
| 2 | Trench align — locks your heading for the trench |
| 3 | Intake in (pivot up) |
| 4 | Intake out (pivot down) |
| 5 | **Stop the roller** — the roller runs on its own, hold this to stop it. Let go and it runs again |
| 6 | Compress by hand |
| 7 | Demo shot — practice and demos only, dead in a match |
| 8 | Bump align — locks your heading for the bump |
| **9** | **Turbo ⚡** — extra push for 2 seconds. See *Turbo* below |
| 10 | Tower shot — fixed shot, no aiming, no waiting |
| 11 | Pass |
| 12 | Cancel auto-X (stop the wheels locking up) |
| Hat, 13–16 | Nothing |

**The Xbox does three things in this mode**

| Xbox | What it does |
|---|---|
| A | Flip which alliance the hub is favouring |
| B | Switch compress to double |
| Y | Turn the hub timer off completely |

Tap them — holding does nothing extra. Every other Xbox button is dead.

---

## MODE 2 — XBOX DRIVES

You hold the Xbox. The flightstick sits on the desk as the backup controller.

```
XBOX ELITE SERIES 2
────────────────────────────────────────────────────────────────────────────────

      STOP ROLLER ── LT                                RT ── SHOOT ★
   INTAKE TOGGLE ── LB                                 RB ── TURBO ⚡
   ╔═════════════════════════════════════════════════════════════════════════╗
   ║                                                                         ║
   ║      ╭─────╮                                              ( Y )         ║
   ║      │  L  │       (  )   (  )   (  )                                   ║
   ║      ╰─────╯                                        ( X )     ( B )     ║
   ║       DRIVE                                                             ║
   ║                                                           ( A )         ║
   ║          ↑                          ╭─────╮                            ║
   ║        ← ✛ →                        │  R  │             Y  TOWER SHOT  ║
   ║          ↓                          ╰─────╯             X  TRENCH      ║
   ║        D-PAD                          TURN              A  BUMP        ║
   ║      ↑ = PASS                    left / right           B  nothing     ║
   ║                                                                         ║
   ╚═════════════════════════════════════════════════════════════════════════╝
      Menu, View, the small centre buttons, clicking the sticks and the
      back paddles all do nothing.
```

| Control | What it does |
|---|---|
| Left stick | Drive |
| Right stick, left / right | Turn |
| **Right trigger** | **Shoot** — aims, spins up and fires. See below. Squeeze it properly, a light pull won't register |
| Left trigger | **Stop the roller** — the roller runs on its own, hold this to stop it. Let go and it runs again |
| **Left bumper** | **Intake toggle** — tap to flip the pivot up, tap again for down |
| **Right bumper** | **Turbo ⚡** — extra push for 2 seconds. See *Turbo* below |
| X | Trench align |
| A | Bump align |
| Y | Tower shot |
| D-pad Up | Pass |
| B | Nothing in this mode |
| Everything else | Nothing |

> **First time driving in Xbox mode:** push the left stick gently forward in open space
> before you trust it in a match. If the robot goes backwards, stop and tell a
> programmer — it's a one-line fix, but not one to discover mid-match.

**The flightstick does five things in this mode**

| Flightstick | What it does |
|---|---|
| 2 | Turn the hub timer off completely |
| 3 or 4 | Flip which alliance the hub is favouring (either button, same result) |
| 6 | Compress by hand |
| 7 | Demo shot — practice and demos only |
| 12 | Cancel auto-X |

Everything else on the flightstick is dead while the Xbox is driving — including the
trigger. **Pulling the flightstick trigger in this mode does nothing.**

---

## What the trigger does

One button. The robot decides what it means from where you are on the field.

```
  IN OUR ZONE, hub is ours          ──►   FULL SHOT
                                          Turns to the hub, spins up, sets the
                                          hood, then fires when it's ready and
                                          lined up. Compress happens on its own.

  IN OUR ZONE, hub is NOT ours     ──►   AIMS BUT WON'T FIRE
                                          It will still turn to face the hub and
                                          then just sit there. This is normal.
                                          It is not jammed. Wait for the hub.

  OUTSIDE OUR ZONE                 ──►   PASS
                                          Turns to your alliance partner's spot
                                          and passes instead of shooting.
```

**Hold it.** All three cases need the trigger held down — it aims, waits, then fires.
Let go early and nothing comes out.

---

## Turbo ⚡

**Flightstick 9 · Xbox right bumper**

Hold it for extra pushing power. It is there for **getting over the bump with a full
hopper**, and for shoving matches. Hold it, drive into the thing, let go once you're over.

```
  IT RUNS FOR 2 SECONDS, THEN STOPS ON ITS OWN.

  Holding the button down longer does NOT keep it going. To get another
  2 seconds you have to LET GO and PRESS IT AGAIN.

  This is deliberate — it stops the motors cooking if you lean on it.
```

- **It does almost nothing at speed.** Turbo is a low-speed, high-shove tool. Driving
  across open carpet you will not feel it, and that's expected — nothing is broken.
- **On open carpet it can spin the wheels.** More power than the tread can hold. If the
  robot feels like it's slipping instead of going, let go of turbo.
- **Use it in bursts.** Don't drive around with your thumb on it.

---

## Good to know

- **The robot slows to half speed while it's aiming.** That is on purpose. It is not a
  dying battery and it is not a brownout.

- **The intake roller runs by itself, the whole match.** There is no button to run it —
  it spins from the moment you enable, in auto and in teleop, and the robot drops it to
  a gentler stir on its own while a shot is going out. Button 5 / LT is the only way to
  stop it: hold to stop, let go to start again. If you stop it mid-shot it won't go back
  to stirring until the next trigger pull.

- **Touching the intake cancels auto-compress.** If you press intake in, intake out, the
  Xbox intake toggle, or compress, the robot stops compressing by itself for the rest of
  that trigger pull. Let go of the trigger and it comes back.

- **The Xbox intake bumper is a toggle, not a hold.** Tap it and the pivot goes up and
  *stays* up. Tap it again to bring it down. On the flightstick they're still two
  separate buttons (3 up, 4 down).

- **Tower shot and demo shot don't aim.** They fire a fixed shot on a timer. Line the
  robot up yourself first.

- **Pass and Trench/Bump align steer for you.** You keep translation control, the robot
  takes the heading. Don't fight it with the twist axis or the right stick.

- **After any redeploy, re-check the mode dropdown.** It resets to Thrustmaster. A
  brownout does *not* reset it.

---

*Button positions confirmed with the drive team. Functions read from the robot code on
28 July 2026 — if a button does something different from this card, the card is wrong,
so tell a programmer.*

*Changed 28 July 2026: turbo added on flightstick 9 / Xbox RB. Xbox intake out moved off
RB — both intake directions are now the LB toggle. Flightstick intake is unchanged.*
