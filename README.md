# Grizzly — Micromouse Module Interaction Simulator

**Live demo:** https://adeyemimiracle.github.io/micromouse-sim/ (GitHub Pages — no login, no expiry)

A self-contained browser simulation of the **Grizzly** micromouse firmware solving an unknown
**16 × 16 maze**. It exists to show, step by step, how each source module hands data to the next
one to turn three ToF readings and two wheel encoders into a shortest-path run to the centre.

## Running it

Open `index.html` in any modern browser (no build step, no server, no dependencies).

- **Run / Pause** — start or freeze the simulated firmware loop.
- **Step action** — execute exactly one action (scan → plan → turn → drive → align).
- **Restart run** — same maze, fresh map. **New maze** — regenerate the maze.
- **Speed** — simulated-time multiplier (1×…16×).
- **Strategy** — Reference round-trip: reach the centre, explore only until a safe route home that
  differs from the outbound route is known, return to start, then sprint the shortest known route back.
  Movement follows the reference flood-fill rule: select the lowest flood value among open neighbors,
  breaking ties in forward, left, right, back order.
- Overlays: flood values, the true (still unknown) maze, sensor beams, trail.
- **MPU yaw loop (gyroAlive)** — uncheck to reproduce the physical Grizzly, whose MPU is mounted so
  the gz yaw channel freezes; the `motion` module then leaves `gyroAlive = false` and `runStraight()`
  falls back to the encoder equaliser + side-wall trim only.

## What you are watching

The right-hand **pipeline** lights up the module that is executing, with the values it is passing
on; the **telemetry** panel mirrors the firmware's serial status line; the **event log** is the
serial capture you would see on the bench.

```
main.ino        boot + pre-flight, then explore → return-to-start → speed-run
   │
VL6180 module   3× ToF (L / F / R), 200 mm range, 999 = no echo
   │
floodfill module  wall map + BFS flood, picks the next cell      ←─ Encoder module (odometry)
   │                                                             ←─ mpu6050 module (heading)
motion module   gyro-closed pivots (dead-gyro time fallback)
distance module runStraight(): soft start + P/I yaw + encoder match + wall-centring trim
   │
MotorControl    signed PWM → TB6612FNG H-bridge → wheels
```

The run follows three phases: **EXPLORE** (optimistic flood toward the centre; after reaching it,
explore only cells on a possible safe route to the start that differs from the outbound route) → **RETURN**
(take the shortest known route that differs from outbound, or the shortest known safe route if the maze has no alternate) →
**SPEED** (flood the known map from the start and take the shortest known route back to the centre).

## Files

| File | Contents |
| --- | --- |
| `js/maze.js` | 16 × 16 maze generation, wall geometry, VL6180 ray cast, wall-contact sliding |
| `js/modules.js` | JS twins of the VL6180, mpu6050, Encoder, MotorControl, distance and motion modules, plus the differential-drive plant |
| `js/solver.js` | the floodfill module (map, BFS, wall vote, wall-centring trim) and the `main.ino` supervisor state machine |
| `js/app.js` | canvas rendering, pipeline/telemetry/log panels, animation clock |

## Adaptations for 16 × 16 (each one commented in the source)

- cell pitch 168 mm → **180 mm**; goal = centre cells (7,7)(7,8)(8,7)(8,8)
- lone-wall standoff 50 mm → **84 mm** (half corridor)
- side-wall threshold 90 mm → **120 mm** (a centred car reads 84 mm; an open side reads 999 at 264 mm,
  so a wider threshold cannot false-trigger)
- turn stop tolerance 5° → **2°**
- wall-trim clamp 40 PWM → **8 PWM** (40 PWM slews this chassis ~100 °/s and over-rotates a cell)
- front-abort disarm 60 mm → **100 mm**
- 90° turns drive **both wheels** (centre pivot); the firmware's single-wheel pivot swings the
  chassis ~45 mm off the corridor every turn
- phase watchdog 90 s → **600 s**
- `ff_updateYawSetpoint()` — declared in the floodfill module, never called by the firmware — re-aligns the
  heading after each segment; an aborted segment is unwound on the encoders so the next wall read
  still belongs to a cell; `ff_moveForwardCells()` drives the measured distance to the next cell
  centre (ideal odometry) instead of a blind fixed step count

## Verified behaviour

The round-trip strategy maps only what it needs to establish a different route, and returns only
through observed corridors. The alternate route may share part of the outbound path, but must use
at least one different edge. After returning
to the start, it plans the sprint using the shortest route available in the mapped maze; that route
may not be the true maze-wide optimum because the strategy deliberately avoids a full sweep. The
dashboard is sized to the screen with its event log scrolling inside its panel rather than the page.

## Using it in a report

Screenshots that tell the story well:

1. early exploration — few known walls (bright cyan), most of the maze still unknown (faint),
   flood values visible;
2. mid-exploration with the pipeline lit on `VL6180 module` → `floodfill module` and the log
   showing the 3-sample wall vote;
3. a turn with `motion module` + `mpu6050 module` lit and the heading closing on 90°;
4. a straight segment with `distance module` / `MotorControl module` / `Encoder module` lit (yaw error,
   correction, wheel trim, PWM, RPM, encoder steps);
5. the alternate return to start followed by the green shortest-known sprint to the centre;
6. the stats *cells explored (centre → sprint)* and *sprint moves / optimal*.
