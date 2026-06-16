---
name: Game Info
description: 2026 FRC game field geometry, zone names, hub locations, and scoring rules as used by the Guerin Robotics robot code.
---

# Game Info — FRC 2026

## Field Dimensions

- Length: ~16.54 m (field-length from AprilTag layout)
- Width: ~8.21 m (field-width from AprilTag layout)
- Origin: blue alliance corner, positive X toward red alliance, positive Y left when facing from blue

## Alliance Coordinate Convention

All `FieldConstants` values are defined from blue alliance perspective. `AllianceFlipUtil.apply()` / `AllianceFlipUtil.shouldFlip()` handles red alliance mirroring. **Never call `DriverStation.getAlliance()` in a hot path** — use the cached `AllianceFlipUtil`.

---

## Hub

The hub is the central scoring structure. Each alliance has one.

| Dimension | Value |
|---|---|
| Width (outer) | 47 in (~1.19 m) |
| Height (outer, including catcher) | 72 in (~1.83 m) |
| Inner width | 41.7 in |
| Inner height | 56.5 in |

**Alliance hub scoring target** (top center): `FieldConstants.Hub.topCenterPoint`
**Opposing hub top center**: `FieldConstants.Hub.oppTopCenterPoint`

Hub faces have AprilTags:
- Blue near face: tag 26
- Blue far face: tag 20
- Blue right face: tag 18
- Blue left face: tag 21

Distance to hub is computed in `RobotState.getDistanceToAllianceHub()` — auto-flips for alliance.

---

## Tower (Climbing)

| Dimension | Value |
|---|---|
| Width | 49.25 in |
| Depth | 45 in |
| Height | 78.25 in |
| Inner opening width | 32.25 in |
| Front face X (from blue wall) | 43.51 in |

Rung heights: low 27 in, mid 45 in, high 63 in.

Alliance tower center AprilTag: 31. Opposing tower: tag 15.

---

## Zone System (used by RobotState)

### Broad Zones (X axis, alliance-relative)

| Zone | Boundary |
|---|---|
| `ALLIANCE_ZONE` | Behind alliance starting line |
| `ALLIANCE_TRENCH` | Alliance starting line → 10 ft from center |
| `NEUTRAL` | 10 ft either side of center |
| `OPPOSING_TRENCH` | 10 ft from center → opposing starting line |
| `OPPOSING_ZONE` | Past opposing starting line |

### Specific Zones (X + Y, alliance-relative)

Within ALLIANCE_TRENCH and OPPOSING_TRENCH:
- `ALLIANCE_TRENCH_NEAR` / `OPPOSING_TRENCH_NEAR` — near (right) trench opening
- `ALLIANCE_BUMP_NEAR` / `OPPOSING_BUMP_NEAR` — near bump area
- `ALLIANCE_HUB` / `OPPOSING_HUB` — directly in front of hub
- `ALLIANCE_BUMP_FAR` / `OPPOSING_BUMP_FAR` — far bump area
- `ALLIANCE_TRENCH_FAR` / `OPPOSING_TRENCH_FAR` — far (left) trench opening
- `ALLIANCE_TOWER` / `OPPOSING_TOWER` — inside tower zone

---

## Bumps and Trenches

**Left Bump** (far/left side of hub):
- Width: 73 in, Depth: 44.4 in, Height: 6.513 in

**Right Bump** (near/right side of hub):
- Width: 73 in, Depth: 44.4 in, Height: 6.513 in

**Left Trench** (far trench, Y > field center):
- Width: 65.65 in, opening width: 50.34 in, opening height: 22.25 in

**Right Trench** (near trench, Y < field center):
- Same dimensions as left trench

---

## Pass Targets

Pass shots are aimed at alliance partner positions. Targets are hardcoded in `RobotState.getPassTarget()` based on robot Y position and alliance:

| Alliance | Robot Y | Target |
|---|---|---|
| Blue | < field center | (4.5 m, 2.3 m) |
| Blue | ≥ field center | (4.5 m, 6.1 m) |
| Red | > field center | (12 m, 6.1 m) |
| Red | ≤ field center | (12 m, 2.3 m) |

---

## Shot Safety Conditions

A shot is only allowed when **both** conditions are true:
- `isShootSafeZone` — robot is not in an unsafe zone (not in opposing tower, etc.)
- `isShootSafeTime` — hub is active per match schedule (`HubShiftUtil`)

`isShootClear` = `isShootSafeZone AND isShootSafeTime`

---

## Depot and Outpost

**Depot** (ball storage on alliance side):
- Width: 42 in, Depth: 27 in
- Center Y offset from field center: 75.93 in

**Outpost** (alliance wall scoring element):
- Width: 31.8 in, opening height: 28.1 in from floor
- Center from AprilTag 29

---

## AprilTag Layout

Layout file: `src/main/deploy/apriltags/welded/2026-official.json`
Tag count: resolved at runtime from `FieldConstants.aprilTagCount`
Tag width: 6.5 in
