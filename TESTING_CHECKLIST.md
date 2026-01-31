# Testing Checklist

Use this checklist to verify the issue creation script works correctly.

## Pre-Flight Checks

- [ ] GitHub CLI is installed (`gh --version`)
- [ ] Authenticated with GitHub (`gh auth status`)
- [ ] Have write access to guerin-robotics/Rebuilt2026
- [ ] Script is executable (`ls -l create_issues.sh`)

## Test Run (Dry Run Recommended)

Before running the full script, you can test creating a single issue manually:

```bash
# Test creating one issue manually
gh issue create \
  --repo "guerin-robotics/Rebuilt2026" \
  --title "TEST - Climber - Define IO Layer" \
  --body "This is a test issue" \
  --label "size: S,priority: Medium"
```

If successful, close the test issue before running the full script.

## Full Script Execution

```bash
./create_issues.sh
```

### Expected Output

The script should:
1. Check for authentication ✓
2. Create 24 issues (6 subsystems × 4 issues each)
3. Display progress for each subsystem
4. Show success message with count

### Verify Results

Visit: https://github.com/guerin-robotics/Rebuilt2026/issues

Check that:
- [ ] 24 new issues exist
- [ ] Issues have correct titles (format: "{Subsystem} - {Phase}")
- [ ] All issues have proper labels:
  - [ ] 12 issues with "size: S"
  - [ ] 12 issues with "size: M"
  - [ ] 6 issues with "priority: Medium"
  - [ ] 18 issues with "priority: Low"
- [ ] Issue bodies contain:
  - [ ] "What" section
  - [ ] "Implementation" section with code examples
  - [ ] "Definition of Done" checklist
  - [ ] "Other Resources" section

## Per-Subsystem Verification

For each subsystem (Climber, Shooter, Hopper, Feeder, Intake, LEDs):

- [ ] **{Subsystem} - Define IO Layer** (S, Medium)
- [ ] **{Subsystem} - Implement RealIO Layer** (M, Low)
- [ ] **{Subsystem} - Implement Basic Subsystem** (M, Low)
- [ ] **{Subsystem} - Implement Basic Subsystem Commands** (S, Low)

## Subsystem Checklist

### Climber
- [ ] Issue 1: Define IO Layer
- [ ] Issue 2: Implement RealIO Layer
- [ ] Issue 3: Implement Basic Subsystem
- [ ] Issue 4: Implement Basic Subsystem Commands

### Shooter
- [ ] Issue 5: Define IO Layer
- [ ] Issue 6: Implement RealIO Layer
- [ ] Issue 7: Implement Basic Subsystem
- [ ] Issue 8: Implement Basic Subsystem Commands

### Hopper
- [ ] Issue 9: Define IO Layer
- [ ] Issue 10: Implement RealIO Layer
- [ ] Issue 11: Implement Basic Subsystem
- [ ] Issue 12: Implement Basic Subsystem Commands

### Feeder
- [ ] Issue 13: Define IO Layer
- [ ] Issue 14: Implement RealIO Layer
- [ ] Issue 15: Implement Basic Subsystem
- [ ] Issue 16: Implement Basic Subsystem Commands

### Intake
- [ ] Issue 17: Define IO Layer
- [ ] Issue 18: Implement RealIO Layer
- [ ] Issue 19: Implement Basic Subsystem
- [ ] Issue 20: Implement Basic Subsystem Commands

### LEDs
- [ ] Issue 21: Define IO Layer
- [ ] Issue 22: Implement RealIO Layer
- [ ] Issue 23: Implement Basic Subsystem
- [ ] Issue 24: Implement Basic Subsystem Commands

## Content Quality Check

Randomly select 3-5 issues and verify:
- [ ] Code examples are present and properly formatted
- [ ] Java syntax is correct in examples
- [ ] Package names use lowercase (e.g., `climber`, not `Climber`)
- [ ] All skeleton examples include helpful comments
- [ ] Definition of Done items are specific and actionable
- [ ] Resources links are valid

## Issue Organization

Check GitHub interface:
- [ ] Issues can be filtered by "size: S"
- [ ] Issues can be filtered by "size: M"
- [ ] Issues can be filtered by "priority: Medium"
- [ ] Issues can be filtered by "priority: Low"
- [ ] Issues are numbered sequentially

## Troubleshooting

If issues fail to create:
1. Check error message from gh CLI
2. Verify authentication: `gh auth status`
3. Verify repository access: `gh repo view guerin-robotics/Rebuilt2026`
4. Check rate limits: GitHub API has rate limits
5. Try creating one issue manually to isolate the problem

## Cleanup (If Needed)

If you need to delete test issues:

```bash
# List all issues
gh issue list --repo guerin-robotics/Rebuilt2026

# Close an issue
gh issue close <issue-number> --repo guerin-robotics/Rebuilt2026

# Or delete if you have admin access
# (Note: Requires admin permissions)
```

## Success Criteria

✅ All 24 issues created
✅ All issues have correct labels
✅ All issues have complete content
✅ Issues are organized and filterable
✅ Code examples are valid
✅ Ready for team to begin implementation

## Post-Creation Tasks

After successful creation:
- [ ] Assign issues to team members (if ready)
- [ ] Create milestone(s) if desired
- [ ] Update project board (if using)
- [ ] Notify team that issues are ready
- [ ] Archive this testing checklist
