# Demo battery simulation (RBT-2000)

> **Simulated, not measured.** The Rover has no battery sensing: no fuel gauge,
> ADC or voltage divider. With this mode on, the percentage on the dashboard is
> computed by the Pi from elapsed time and distance. It says nothing about the
> real pack. The mode is off by default and exists only for demonstrations.

Code: `robotx/hardware/demo_battery.py`. Tests: `tests/unit/test_demo_battery.py`.

## What it does

1. A fresh demo session starts at **100 %**.
2. The percentage drains with the **backend simulator's own formula**, by
   elapsed time and, on a mission, by distance driven. See
   [Drain](#drain-audited-from-the-backend-simulator) below.
3. At or below **30 %**, and only while the Rover is idle (mode `IDLE`, no
   mission held), a **demo recharge** starts. The percentage is held for
   **60 s**, then set to **100 %**.
4. Drain resumes, and the cycle repeats.

The recharge is a **demo shortcut**. It is not the backend simulator's charging
model (charger docking plus a CC/CV curve, `chargeModel.js`), and it controls no
hardware. A mission that arrives during the minute cancels the recharge; it
restarts, for a full minute, the next time the Rover is idle.

## Drain (audited from the backend simulator)

Source: RobotX repository, `Backend/src/simulation/`.

**`power` model (default).** Every simulated robot commissioned under the
current power policy uses it (`energyPolicy.js`; `VirtualRobot._applyBattery`,
basis `SIMULATED_POWER_WH`):

```
beta_aux  = P_standby / 3600                         Wh per second, always
beta_dist = (P_moving - P_standby) / (3600 * v_ref)  Wh per metre driven
Wh        = beta_aux * seconds + beta_dist * metres
percent  -= 100 * Wh / packNominalWh                 floored at 5 %
```

The constants:

| Constant | Value | Source |
|---|---|---|
| `P_standby` | 9.5 W | `energyPolicy.js` |
| `P_moving` | 30 W at `v_ref` = 2 m/s | `energyPolicy.js` |
| `packNominalWh` | 1000 Wh | `constants.js` `PACK_NOMINAL_WH` |
| Floor | 5 % | `constants.js` `BATTERY_MIN` |

`beta_move_time` and `beta_dwell` are 0, so travel differs from idle only by
the distance term.

**`legacy_percentage` model.** This is the simulator's fallback for robots
whose energy row is incomplete (`constants.js`):

```
percent -= rate * seconds / 2 s                      floored at 5 %
rate = 0.0444 %/tick working (ACTIVE / RETURNING / ISSUES on a task)
rate = 0.0089 %/tick otherwise (IDLE / PAUSED)
```

**How long until the 30 % recharge:**

| Model | Idle | Driving |
|---|---|---|
| `power` | 0.95 %/h, so ~74 h | at 0.5 m/s: 14.6 W = 1.46 %/h, ~48 h |
| `legacy_percentage` | 16.0 %/h, so ~4.4 h | 79.9 %/h, ~53 min |

With `power` the value barely moves during a demo. Choose
`legacy_percentage` if the drain and recharge cycle should be visible. No
faster rate is offered, because the backend has none.

**Where the Pi differs from the simulator:**
- **Distance.** It is the great-circle distance between consecutive *measured*
  GPS fixes. It is counted only while a mission is driven (mode `AUTO`), so
  stationary GPS jitter is not drain. A single jump over 500 m is ignored,
  which is the backend's own bound for a real robot's distance.
- **Pauses.** The step is real elapsed time from a monotonic clock, so the loop
  rate does not change the drain. One step is capped at 10 s, so a stalled loop
  or suspended process cannot cause a jump. A clock going backwards drains
  nothing.

## How it reaches the dashboard

The Pi uses the **existing** `TELEMETRY` event and its existing `battery`
field. That is the field the backend's `telemetry.handler.js` already parses,
caches, broadcasts and writes to `Robot.battery`. There is no new event, no new
endpoint and no backend change.

- **Simulation off:** `battery` is never sent, exactly as before. The dashboard
  shows "0%" only because `Robot.battery` is `null` and the frontend renders
  `battery || 0`.
- **Simulation on:** every frame carries `battery` (0.1 % resolution).
- **`energy.socMethod` is never sent.** Without it the backend stores the value
  for display but never writes it into `BatteryState.lastObservedSoc`
  (`batteryObservation.service.js`, P2B-2). So the assignment engine never plans
  with the simulated value. Physical dispatch eligibility is unchanged: it still
  uses the operator-declared state of charge.
- **The recharge is reported as `status: "CHARGING"`.** The backend's §23.5
  trust boundary refuses any frame whose state of charge **rose** unless it says
  `CHARGING`. Where the engine enforces that check, it drops the whole frame,
  GPS included. Every later frame is then refused too, because the comparison
  value never advances.

  So the Pi sends `CHARGING` (an existing backend status; it is never written to
  `Robot.status`) for the recharge minute and for the first 100 % frame. It
  holds drain at 100 % until that frame has actually left the Pi. If a mission
  is held at that moment, `battery` is left out of the frame rather than sent
  without `CHARGING`. A fresh or unreadable state file starts the same way, so
  the first 100 % is never a refused rise.

  During the recharge minute the dashboard shows the robot as **CHARGING**.

## Enable on the Pi

Add the following to `/etc/robotx/robotx-agent.env`:

```bash
ROBOTX_DEMO_BATTERY_ENABLED=1
# Optional (defaults shown):
# ROBOTX_DEMO_BATTERY_DRAIN_MODEL=power      # or legacy_percentage
# ROBOTX_DEMO_BATTERY_INITIAL_PERCENT=100
# ROBOTX_DEMO_BATTERY_RECHARGE_THRESHOLD_PERCENT=30
# ROBOTX_DEMO_BATTERY_RECHARGE_S=60
# ROBOTX_DEMO_BATTERY_STATE_PATH=~/.robotx/demo_battery.json
```

Then restart the service:

```bash
sudo systemctl restart robotx-agent
```

An invalid value (for example a threshold ≤ 5 or ≥ 100, or an unknown model)
**disables the simulation**, not the robot. It is logged as
`demo_battery.config_invalid`, and the battery stays UNAVAILABLE.

## Persistence and restarts

State is one JSON file, by default `~/.robotx/demo_battery.json` in the service
account's home directory. It is written atomically (temp file and rename).

It is rewritten on every phase change and whenever the 0.1 % value changes, so
a restart never resumes above the last value the backend saw. It is also
rewritten once a second during a recharge, so a restart mid-recharge keeps the
timer.

- **Restart.** The battery resumes from the file. Downtime is not drained, and
  the battery is never reset to 100 %.
- **Missing file.** This is a fresh session at 100 %.
- **Unreadable or invalid file** (bad JSON, unknown version or phase, a
  percentage outside 0–100). The file is logged as `demo_battery.state_invalid`
  and replaced by a fresh 100 % session, which is reported as a recharge.
- **Starting a fresh demo session:**

  ```bash
  sudo systemctl stop robotx-agent
  rm ~<service-user>/.robotx/demo_battery.json
  sudo systemctl start robotx-agent
  ```

- **No persistence.** If `ROBOTX_DEMO_BATTERY_STATE_PATH` is set to an empty
  value, the state is kept in memory only, and every restart is a fresh
  session.

## Verify

On the Pi (local API, see the operator runbook):

```bash
S=http://127.0.0.1:8000
curl -s $S/state | jq .power        # status "SIMULATED", percent, charging, phase
curl -s $S/telemetry | jq .battery  # the same value
journalctl -u robotx-agent | grep demo_battery   # enabled / phase / resumed / invalid
cat ~<service-user>/.robotx/demo_battery.json
```

On the backend and dashboard:
- The robot's battery on the Robots page and the robot detail page shows the
  simulated percentage instead of 0%.
- The backend log must **not** show `TELEMETRY refused by the §23.5 trust
  boundaries` for RBT-2000. If it does, the reported `reasons` say why.
- During a recharge minute, the robot's status shows `CHARGING`.

## Disable (restore the hardware path)

Remove the line, or set `ROBOTX_DEMO_BATTERY_ENABLED=0`, then restart the
service. `battery` is no longer sent, and `/state` shows `power.status:
"UNAVAILABLE"` again.

The backend keeps the last displayed value in `Robot.battery` until something
else writes it. Nothing on the Pi clears it.

## Limitations

- The percentage is a simulation of the backend's model, not of this pack.
- Drain is applied only while the agent runs. Time with the service stopped is
  not drained.
- If the state file cannot be written (for example a full disk; logged as
  `demo_battery.persist_failed`), a restart can resume slightly above the last
  value reported. The backend then refuses frames as a "rise" until the
  simulation drains back below that value.
- The Rover's `assess_offer` has no battery check, so offers are answered
  exactly as before. The simulated battery never makes the Rover refuse or
  accept work.
