## WPILib Agent Tools

Use `{{RUNNER_PATH}}` instead of ad-hoc shell/python glue when you need WPILib-specific simulation, log, or validation workflows.

Guidance:
- Start with `{{RUNNER_PATH}} --help` and `{{RUNNER_PATH}} <subcommand> --help` before assuming flags.
- Prefer sandbox/sim/log/query workflows through the tool when validating robot-code changes.
- Prefer bounded outputs such as `--json`, `--summary`, `--limit`, and `--max-lines` before verbose dumps.
- When validating a robot repo end-to-end, prefer `{{RUNNER_PATH}} sandbox ...` or the repo validation scripts that ship with this toolchain.
