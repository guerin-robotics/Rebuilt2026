## WPILib Agent Tools

Use `{{RUNNER_PATH}}` for WPILib-specific simulation, log, and validation workflows.

Guidance:
- Start with `{{RUNNER_PATH}} --help` and `{{RUNNER_PATH}} <subcommand> --help` before assuming flags.
- Prefer the tool's sandbox/sim/log/query workflows over custom shell/python glue when validating robot behavior.
- Prefer bounded outputs such as `--json`, `--summary`, `--limit`, and `--max-lines` before verbose dumps.
