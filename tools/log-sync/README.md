# log-sync — auto-upload roboRIO logs to Google Drive

Pulls new `.wpilog` files off the roboRIO **over the network** and uploads them to
Google Drive. The robot's USB stick never has to be plugged into a computer.

```
roboRIO /U/logs  --(SFTP)-->  this laptop  --(HTTPS)-->  Google Drive
                              (streamed, nothing saved to local disk)
```

`rclone` only transfers files Drive doesn't already have, so the newest logs are
always uploaded and re-running is cheap.

**Requirement:** the laptop running this must reach *both* the robot and the
internet at once. In the shop that's constant. At a competition the roboRIO is
firewalled off the internet on the field, so uploads happen from the pit network
between matches — not live during a match.

---

## One-time setup

### 1. Install rclone

```bash
brew install rclone
```

### 2. Configure the Google Drive remote (interactive, opens a browser)

```bash
rclone config
```

- `n` (new remote) → name it **`gdrive`**
- Storage: **`drive`** (Google Drive)
- Leave client_id / client_secret blank (press Enter)
- Scope: **`1`** (full access) or `3` (drive.file — only files this tool creates)
- Accept defaults, choose **`y`** to auto-authenticate — a browser opens, log in,
  allow access.

### 3. Configure the roboRIO SFTP remote

```bash
rclone config
```

- `n` (new remote) → name it **`roborio`**
- Storage: **`sftp`**
- **host:** your roboRIO — `10.TE.AM.2` (replace `TE.AM` with your team number, e.g.
  team 1234 → `10.12.34.2`), or `roboRIO-1234-FRC.local`
- **user:** `lvuser`
- **port:** blank (22)
- For auth, the roboRIO's `lvuser` has no password. Easiest reliable option is an
  SSH key (see below). If you skip the key, choose to enter a password and leave
  it **blank**.

**Recommended — passwordless SSH key (do this once):**

```bash
# generate a key if you don't have one
ssh-keygen -t ed25519 -f ~/.ssh/roborio -N ""

# install it on the robot (robot must be on and reachable)
ssh-copy-id -i ~/.ssh/roborio.pub lvuser@10.TE.AM.2
```

Then in the `roborio` rclone remote set **key_file** to `~/.ssh/roborio`.

### 4. Test it

```bash
rclone lsf roborio:/U/logs           # should list the .wpilog files on the robot
./tools/log-sync/sync-logs.sh        # one sync; check the FRC-Logs folder in Drive
```

---

## Running it

Sync once:

```bash
./tools/log-sync/sync-logs.sh
```

Keep syncing every 2 minutes while you work:

```bash
./tools/log-sync/sync-logs.sh --watch
```

### Run automatically in the background (macOS launchd)

Edit `com.guerin.logsync.plist` if you moved the repo, then:

```bash
cp tools/log-sync/com.guerin.logsync.plist ~/Library/LaunchAgents/
launchctl load ~/Library/LaunchAgents/com.guerin.logsync.plist
```

It starts on login and runs `sync-logs.sh --watch`. Logs go to
`/tmp/guerin-log-sync.out`. To stop:

```bash
launchctl unload ~/Library/LaunchAgents/com.guerin.logsync.plist
```

---

## Options (environment variables)

| Var          | Default      | Meaning                                        |
|--------------|--------------|------------------------------------------------|
| `DST_PATH`   | `FRC-Logs`   | Google Drive destination folder                |
| `INCLUDE`    | `*.wpilog`   | File filter — set to `{*.wpilog,*.hoot}` for CTRE Hoot logs too (large) |
| `INTERVAL`   | `120`        | Seconds between syncs in `--watch` mode        |
| `SRC_PATH`   | `/U/logs`    | Log directory on the roboRIO                   |

Example: also grab Hoot logs, check every 5 min:

```bash
INCLUDE='{*.wpilog,*.hoot}' INTERVAL=300 ./tools/log-sync/sync-logs.sh --watch
```

---

## Notes

- `.hoot` (CTRE signal) logs are large; they're excluded by default. Include them
  deliberately if you have the bandwidth.
- This is laptop-side tooling only — it touches nothing under `src/` and does not
  affect robot behavior.
