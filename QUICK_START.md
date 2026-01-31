# Quick Start Guide - Creating GitHub Issues

## Step 1: Authenticate with GitHub

```bash
gh auth login
```

Follow the prompts to authenticate with your GitHub account.

## Step 2: Run the Issue Creation Script

```bash
./create_issues.sh
```

This will create all 24 issues automatically.

## Step 3: Verify Issues Were Created

Visit: https://github.com/guerin-robotics/Rebuilt2026/issues

You should see 24 new issues organized by subsystem.

## What Gets Created

### 6 Subsystems × 4 Issues Each = 24 Total Issues

**For each subsystem (Climber, Shooter, Hopper, Feeder, Intake, LEDs):**

1. **Define IO Layer** - Size: S, Priority: Medium
   - Creates the hardware abstraction interface
   - Defines sensor inputs and control methods
   
2. **Implement RealIO Layer** - Size: M, Priority: Low
   - Implements actual hardware using CTRE Phoenix 6
   - Configures TalonFX motors
   
3. **Implement Basic Subsystem** - Size: M, Priority: Low
   - Creates the subsystem class
   - Adds periodic logging and basic control
   
4. **Implement Basic Subsystem Commands** - Size: S, Priority: Low
   - Creates operator control commands
   - Enables testing and teleoperation

## Troubleshooting

### "gh: command not found"
Install GitHub CLI: https://cli.github.com/

### "You are not logged into any GitHub hosts"
Run: `gh auth login`

### "Permission denied"
Make the script executable: `chmod +x create_issues.sh`

### "Resource not accessible by integration"
Ensure you have write access to the repository

## Alternative: Manual Issue Creation

If automated creation doesn't work, you can:
1. Reference `ISSUES_README.md` for detailed issue content
2. Use the GitHub web interface to create issues manually
3. Copy content from the script for each issue

## Next Steps

After issues are created:
1. Assign issues to team members
2. Add milestones if desired
3. Begin implementation following the issue dependencies
4. Start with IO Layer definitions (Priority: Medium)

## Support

For help with:
- Script issues: Check the script comments
- GitHub CLI: https://cli.github.com/manual/
- Project questions: Contact team leads
