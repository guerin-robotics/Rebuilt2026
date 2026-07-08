# Prompt: Write a Test

Use this prompt to capture a behavior or a fixed bug in an automated JUnit test.
Fill in all bracketed values. The known-correct cases are YOUR job — they come from
physics, measurements, or game rules, not from the code. A test derived only from
reading the code just proves the code equals itself.

---

```
Write a JUnit test for [CLASS or BEHAVIOR, e.g. ShotCalculator distance→RPM interpolation].

Known-correct cases (input → expected output; include edge cases):
- [e.g. 3.0 m → 2400 RPM, 34° hood — measured at practice 2026-XX-XX]
- [e.g. 0.0 m → clamped to minimum table entry, no exception]
- [e.g. beyond max table distance → clamped to last entry]

[If this is a regression test for a fixed bug:]
Bug being locked in: [what happened, e.g. "single-tag poses beyond 4 m with
ambiguity > 0.2 were accepted and caused pose jumps"]
The test must FAIL on the pre-fix code and PASS on the current code.

Rules:
- Test goes in src/test/java, mirroring the main package structure
- No hardware objects — test pure logic, or use the IOSim implementation
- Every test method asserts something specific; no smoke-only tests
- After writing it, prove the test can fail: temporarily break the behavior,
  run the test, show me the failure output, then revert the break
- Run ./gradlew build and show the test results before reporting complete
```

---

## Safety Notes

- Writing a test is a Safe-tier change — but if writing the test requires
  refactoring production code for testability, STOP and surface that as a
  separate, higher-risk change. Do not restructure working robot code as a
  side effect of adding a test.
- A regression test is part of the bug fix. The fix is not "done" until the
  test exists and is green in CI.
- If the expected values are guesses rather than measurements, say so in a
  comment in the test — a test asserting a guess locks in the guess.
