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
- **Speed** — simulated-time multiplier (1×…16×). A full-map run is ~9 minutes of simulated time:
  ~7.5 min of exploration (256 cells), then the return to start, then the speed run.
- **Strategy** — *Goal-first (reach goal, then optimise)* (default) or
  *Full-map exploration*; see below. Switching it restarts the run.
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

Phases, exactly as in `runFloodFillSequence()`: **EXPLORE** → **RETURN** (flood to the start cell)
→ **SPEED** (flood to the centre 2 × 2 and drive the shortest path, drawn in green) → **DONE**.
Both strategies walk the same phases; they differ in how long EXPLORE runs and what it aims at.

## Two run strategies

Both are legal micromouse strategies; both end with a speed run from the start cell to the centre.
They differ only in how much of the maze is mapped first.

**Full-map exploration.** EXPLORE floods toward the nearest still-unvisited cell and does not stop
until every reachable cell (256) has been mapped. The sprint is then planned from a complete map,
so its route is the true shortest path — but the run takes ~8–9 minutes of simulated time.

**Goal-first — "reach goal, then optimise"** (default). The mouse never maps the whole maze, and it
does **not** turn round the moment it touches the centre either — the corridors it came in on are
rarely the fastest ones. It runs in four steps:

1. **Drive at the centre.** EXPLORE floods straight at the goal, treating unknown cells as open, and
   re-plans whenever a wall turns up.
2. **Keep exploring around the goal.** After first contact it keeps sweeping the cells that could
   still shorten the route: a cell is worth visiting only while the shortest route that *could* run
   `start → cell → goal` (unknown cells open, known walls respected) is shorter than the best route
   the map already proves. Those cells are ranked by how far the car must drive to reach them, so the
   sweep starts with the goal's own surroundings and works outward. It stops when no unexplored cell
   can beat the known route — or after 120 extra cells, so a maze where every cell still looks
   promising cannot turn into a full sweep.
3. **Return to the start by the shortest *known* route** — by then usually a **different** route from
   the way in, because step 2 mapped the alternatives.
4. **Sprint** the best route in the map (green path).

The return and the sprint are routed only through corridors the mouse has actually observed (scanned
or driven), so an unknown wall can never stop the sprint. The price is that the route is optimal
**only for the known map**, so it is occasionally a few cells longer than the true optimum. The stats
panel shows both numbers: *cells explored (centre → sprint)* and *sprint moves / optimal* (the
optimal figure is the BFS shortest path through the true maze, shown as a yardstick).

Measured over 16 generated mazes (headless runs):

| | Goal-first (reach goal, then optimise) | Full-map exploration |
| --- | --- | --- |
| cells mapped before the sprint | 21 – 167 (mean 72) | 256 |
| — of those, before the centre was first reached | 20 – 91 (mean 48) | 34 – 205 (mean 96) |
| — of those, while exploring around the goal | 0 – 133 (mean 24) | 51 – 232 (mean 160) |
| simulated run time | 52 – 255 s (mean 149 s) | 491 – 530 s (mean 510 s) |
| distance driven | 11 – 47 m (mean ≈ 29 m) | 92 – 99 m (mean ≈ 95 m) |
| sprint vs true optimum | **equal in 16/16** | equal in 16/16 |
| return route is the same as the way in | 4 / 16 (and often much shorter: 96→13, 56→17, 35→10 cells) | 0 / 16 |
| goal reached | 16 / 16 | 16 / 16 |

In other words: goal-first maps ~28 % of the maze and takes ~29 % of the time and distance of a full
sweep, and with the post-goal sweep it still sprints the true optimal route in every maze tested.
Without that sweep (the previous "stop dead on the centre" version) it only matched the optimum in
10/16, because it had never looked at the corridors it did not happen to drive through.

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

Headless runs over several generated mazes:

- *Full-map exploration*: **256/256 cells mapped**, `allVisited()` true, goal reached in every case
  (0–2 aborted segments, no stalls, ~490–580 moves, ~90–107 m driven).
- *Goal-first*: goal reached in **16/16** mazes, 21–167 cells mapped, 0–3 aborted segments,
  no stalls or oscillations; the sprint equals the true optimum in 16/16.

## Using it in a report

Screenshots that tell the story well:

1. early exploration — few known walls (bright cyan), most of the maze still unknown (faint),
   flood values visible;
2. mid-exploration with the pipeline lit on `VL6180 module` → `floodfill module` and the log
   showing the 3-sample wall vote;
3. a turn with `motion module` + `mpu6050 module` lit and the heading closing on 90°;
4. a straight segment with `distance module` / `MotorControl module` / `Encoder module` lit (yaw error,
   correction, wheel trim, PWM, RPM, encoder steps);
5. the speed run — green shortest path (with goal-first, note how much of the maze is still
   unknown: that is the whole point of the strategy);
6. the same maze run twice, once per strategy, with the two stats *cells explored (centre → sprint)*
   and *sprint moves / optimal* side by side — a compact argument for the strategy choice;
7. the module map table at the bottom for the architecture section.
