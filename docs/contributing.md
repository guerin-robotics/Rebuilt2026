# Contributing & Code Review

*This document exists because people graduate. Every rule here is designed so that a new member
in three years can pick up this codebase and contribute safely — without needing a veteran
to explain every unwritten convention.*

---

## Why This Matters

FRC teams lose institutional knowledge every spring. Without enforced standards, each
generation inherits code no one fully understands, style that drifts with every new
contributor, and changes that were never reviewed. The rules below protect the robot
and the next team, not just today's build.

---

## Branch & Merge Rules

**No one pushes directly to `main`.** Every change goes through a pull request.

**Maintainers (merge authority):** 2 designated leads per season — typically the software
captain and one senior member. Only they may approve and merge PRs.  
Update this list in `.github/CODEOWNERS` at the start of each season.

| Branch type | Naming | Merges into |
|---|---|---|
| Feature / new mechanism | `feature/description` | `main` |
| Bug fix | `fix/description` | `main` |
| Tuning session | `tune/subsystem-name` | `main` |
| Release prep | `release/YYYY-event-name` | `main` → tagged |

**All PRs require:**
- [ ] Passing `./gradlew compileJava` and `./gradlew spotlessCheck`
- [ ] AI review (see below)
- [ ] At least one human maintainer approval

---

## AI Review (Required on Every PR)

Before requesting human review, run `/ultrareview` in Claude Code on your branch.
Paste the summary as a comment on the PR.

The AI review checks architecture rules, safety interlocks, scope creep, and logging
consistency per [CLAUDE.md](../CLAUDE.md). It does not replace human judgment — it
catches the mechanical violations so the human reviewer can focus on intent.

---

## Releases & Tagging

Tag every robot that competes. Tags are how future members match a log file to source code.

```
Format:  v<year>-<event-code>[-rc<n>]
Example: v2026-week1, v2026-champs, v2026-champs-rc2
```

**Before tagging:**
1. Confirm `Constants.getRobotType()` == `COMP`
2. Confirm `DEMO_MODE` and `TUNING_MODE` are `false`
3. Run `./gradlew build` — must pass clean

Tag on `main` after merging, not on a feature branch. Push the tag explicitly:
```bash
git tag v2026-champs && git push origin v2026-champs
```

---

## Code Style

This repo uses **Google Java Format** enforced by Spotless.

```bash
./gradlew spotlessApply   # auto-format before committing
./gradlew spotlessCheck   # verify in CI
```

Beyond formatting, consistency rules enforced by [CLAUDE.md](../CLAUDE.md):
- Commands are static factories with `.withName()` — never `extends Command`
- Hardware lives behind `XxxIO` interfaces — never in subsystem classes
- Constants belong in `HardwareConstants` or `[Subsystem]Constants` — never inline
- No `System.out.println()` — use `Logger.recordOutput()`

When in doubt, look at how an existing subsystem does it and match it exactly.

---

## Commit Messages

One logical change per commit. Message explains **why**, not what.

```
GOOD: "Increase feeder timeout to handle slow auto-aim at far distances"
BAD:  "Changed alignmentTimeoutSeconds from 0.9 to 1.1"
```

See [review-checklist.md](review-checklist.md) for the full pre-merge checklist.
