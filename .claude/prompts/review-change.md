# Prompt: Review a Proposed Change

Use this prompt before implementing a significant change, or to get a second opinion
on a change someone else proposed.

---

```
Review this proposed change for safety and correctness:

File(s) to change: [list files]

Proposed change:
[paste the diff or describe the change in detail]

My concern: [what you're unsure about — optional but helpful]

Please evaluate:
1. Safety — does this change any Hard Stop items? (CAN IDs, gains, safety interlocks)
2. Behavioral consequence — what does the robot do differently after this change?
3. Failure mode — what goes wrong if this change is incorrect?
4. Scope — does this change more than necessary to achieve the goal?
5. Alternatives — is there a smaller or safer way to achieve the same outcome?
```

---

## What to Expect in a Review Response

A good review response will:

1. **Classify the risk level** (Safe / Low / Medium / High / Blocked per `00-safety.md`)
2. **State the behavioral change** explicitly — what the robot does differently
3. **Name the failure mode** if the change is wrong
4. **Flag any scope creep** — changes that weren't requested
5. **Recommend proceed, modify, or stop**

A review response should NOT:
- Say "looks good" without analysis
- Approve a High-risk change without naming the failure mode
- Approve a Hard Stop change without explicit user confirmation

---

## Self-Review Checklist (Run Before Asking for Review)

```
Architecture:
[ ] Does not add hardware calls to a subsystem class
[ ] Does not create a subsystem-to-subsystem reference
[ ] Does not create a new command class (uses static factory)
[ ] Does not remove Logger.processInputs() from periodic()

Safety:
[ ] Does not change any CAN IDs
[ ] Does not change any motor inversion flags
[ ] Does not change swerve encoder offsets
[ ] Does not remove a safety timeout
[ ] Does not raise a current limit without explanation

Scope:
[ ] Only changes what was requested
[ ] No opportunistic reformatting or renaming
[ ] No unrelated file edits

Build:
[ ] ./gradlew compileJava passes
[ ] ./gradlew spotlessCheck passes
[ ] No debug System.out.println() left in
```
